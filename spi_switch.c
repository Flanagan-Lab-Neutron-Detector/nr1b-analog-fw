// SPI master and slave!

// stdlib
#include <stdio.h>
#include <string.h>

// pico libraries
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/adc.h"

// GPIO defines
#define CS_DAC0          9  // active low
#define ALARM_DAC0       14 // active low, shows when tempcal is done
#define CS_DAC1          12
#define ALARM_DAC1       15
#define LDAC             13 // register -> dac output, active low, enable with LDACMODE
#define ACK              2  // acknowledge to controller
#define ERROR            3  // error to controller
#define DAC0_READPIN     26 // GPIO pin
#define DAC0_READADC     0  // ADC pin
#define DAC1_READPIN     27
#define DAC1_READADC     1

#define BASICDAC //test option; receiving any SPI transfer will give the DAC a setting
#undef BASICDAC

// Initializing buffers
#define BUF_LEN         0x04 // 4 bytes to trigger interrupt
#define EMPTY_LEN       0x01 // single byte to clear buffer
#define DAC_LEN         0x04 // always 4 bytes, 32 bits

uint8_t in_sys_buf[BUF_LEN], out_sys_buf[BUF_LEN], in_dac_buf[BUF_LEN], out_dac_buf[BUF_LEN], in_command_buf[BUF_LEN];
uint8_t empty_buf, null_buf[EMPTY_LEN];//for flushing buffers

float dac0_scale = 1.0; //uV per bit, important for calibration
float dac1_scale = 1.0;

float adc0_scale = 1.0; //bit per uV
float adc1_scale = 1.0;

// Message frames

struct analog_set_frame {
    bool rw : 1;
    uint8_t address : 4;
    uint32_t microvolts : 23;
    uint8_t reserved : 3;
    bool checksum : 1;
};

struct reset_frame {
    bool rw : 1;
    uint8_t address : 4;
    bool mcu : 1;
    bool dac0 : 1;
    bool dac1 : 1;
    uint32_t reserved : 23;
    bool checksum : 1;
};

struct analog_get_frame {
    bool rw : 1;
    uint8_t address : 4;
    uint32_t microvolts: 23;
    uint8_t reserved : 3;
    bool checksum : 1;
};

/*
struct dac_data_frame {
    bool rw : 1;
    uint8_t address : 7;
    uint32_t dac_setting : 20;
    uint8_t reserved0 : 4;
};
*/
struct dac_data_frame {//get that order reversed!
    uint8_t reserved0 : 4;
    uint32_t dac_setting : 20;
    uint8_t address : 7;
    bool rw : 1;
};

struct dac_config1_frame {
    bool rw : 1;
    uint8_t address : 7;
    bool en_tmp_cal : 1;
    uint8_t reserved0 : 3;
    uint8_t tnh_mask: 2;
    uint8_t reserved1 : 3;
    bool ldac_mode : 1;
    bool fsdo : 1;
    bool enalmp : 1;
    bool dsdo : 1;
    bool fset : 1;
    uint8_t vrefval : 4;
    bool reserved2 : 1;
    bool pdn : 1;
    uint8_t reserved3 : 4;
};

struct dac_trigger_frame {
    bool rw : 1;
    uint8_t address : 7;
    uint16_t reserved0 : 15;
    bool rcltemp : 1;
    bool reserved1 : 1;
    bool srst : 1;
    bool sclr : 1;
    bool reserved2 : 1;
    uint8_t reserved3 : 4;
};

union dac_data {
    struct dac_data_frame setting;
    uint8_t frame_buf[DAC_LEN];
};

union dac_config1 {
    struct dac_config1_frame setting;
    uint8_t frame_buf[DAC_LEN];
};

union dac_trigger {
    struct dac_trigger_frame setting;
    uint8_t frame_buf[DAC_LEN];
};

union analog_set_data {
    struct analog_set_frame setting;
    uint8_t frame_buf[BUF_LEN];
};

union reset_data {
    struct reset_frame setting;
    uint8_t frame_buf[BUF_LEN];
};

union analog_get_data {
    struct analog_get_frame setting;
    uint8_t frame_buf[BUF_LEN];
};

uint8_t data_frame_buf[BUF_LEN] = { 
    0x01,
    0x7F,
    0xFF,
    0xF0
};

uint8_t config_frame_buf[BUF_LEN] = {
    0x02,
    0x00,
    0x1C,
    0x80
};

union dac_data dac_data_0, dac_data_1;
union dac_config1 dac_config1_0, dac_config1_1;
union dac_trigger dac_trigger_0, dac_trigger_1;

union analog_set_data sys_analog_set;
union reset_data sys_reset;
union analog_get_data sys_analog_get;



bool command_received = false;
bool reply_pending = false;
bool command_pending = false;
uint8_t selected_dac = 0;
uint32_t raw_voltage_in; // we'll convert this to a DAC setting


bool parseCommand(void);
bool generate_parity(uint8_t frame[DAC_LEN]);
void spiInit(void);
void printbuf(uint8_t *buf, size_t len);
void dacInit(void);

uint32_t readADC(uint8_t id);

uint32_t last_adc0 = 24;
uint32_t last_adc1 = 56;


void onSpiInt(void) // ISR from SSPRXINTR
{
    // Since we have at least 4 bytes in our buffer (interrupt has fired) we read 4.
    spi_read_blocking(spi0, empty_buf, in_sys_buf, BUF_LEN); // Read buffer in, send 0s back.
    // Now we clear the rest of the buffer.
    while(spi_is_readable(spi0)) {
        spi_read_blocking(spi0, empty_buf, null_buf, EMPTY_LEN); // Read buffer into nowhere, send 0s back.
    }
    command_received = true; // Set the flag, will be processed by main loop.
}


int main(void)
{
    // Enable UART so we can print
    stdio_init_all();

    adc_init();
    //set up LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    //status indication
    gpio_init(ACK);
    gpio_init(ERROR);
    gpio_set_dir(ACK, GPIO_OUT);
    gpio_set_dir(ERROR, GPIO_OUT);
    adc_gpio_init(DAC0_READPIN);
    adc_gpio_init(DAC1_READPIN);

    spiInit(); //set up SPI peripherals and CS pins

    dacInit();
    

    while (1) //main loop
    {
        if (command_received){//ISR has fired!
            memcpy(in_command_buf, in_sys_buf, BUF_LEN);//we get this out of in_sys_buf because that is acted upon by ISR
            command_received = false;
            //time to process our command

            printf("Received the following command: ");
            printbuf(in_command_buf, BUF_LEN);

            //parse command, verify, interpret, choose next step.
            command_pending = parseCommand(); //will always fire in current state
            //parseCommand will return 1 if we need to send something out to DAC.
        }

        if (reply_pending) {//need to send data back to controller
            //things to respond with go into out_sys_buf
            spi_write_read_blocking(spi0, out_sys_buf, in_sys_buf, BUF_LEN); //write DAC response, get new command
            printf("Replied, got new command: ");
            printbuf(in_sys_buf, BUF_LEN);
        }

        if (command_pending) {
            //we need to send something out to DAC
            switch (selected_dac) {
                case 0:
                    gpio_put(CS_DAC0, 0);
                    //spi_write_blocking(spi1, dac_data_0.frame_buf, DAC_LEN);
                    //flip it around before sending
                    for(int i = 0; i < BUF_LEN; i++){
                        data_frame_buf[i] = dac_data_0.frame_buf[(BUF_LEN - 1) - i];
                    }
                    spi_write_blocking(spi1, data_frame_buf, DAC_LEN); //write the base config

                    gpio_put(CS_DAC0, 1);
                    printf("Wrote to DAC0\n");
                    break;
                case 1:
                    gpio_put(CS_DAC1, 0);
                    spi_write_blocking(spi1, dac_data_1.frame_buf, DAC_LEN);
                    gpio_put(CS_DAC1, 1);
                    break;
                default:
                    break;
            }
            command_pending = false;
        }
    }

    return 0;
}


//return 1 if we need to reply
bool parseCommand (void)
{
    printf("parsing command \n");
#ifdef BASICDAC //just some debug/testing stuff
    if (in_command_buf[3] != 0x00) {
        //let's assume we want to set DAC0 to 0x7FFFF (midscale). 
        dac_data_0.setting.dac_setting = 0x803FF;//7FFFF + 1024d
        selected_dac = 0;
        for(int i = 0; i < BUF_LEN; i++){
            data_frame_buf[i] = dac_data_0.frame_buf[(BUF_LEN - 1) - i];
        }
        printf("setting to midrange\n");
        printf("Original buffer: ");
        printbuf(dac_data_0.frame_buf, DAC_LEN);
        //memcpy(data_frame_buf, in_command_buf, BUF_LEN);
        printf("Flipped buffer: ");
        printbuf(data_frame_buf, DAC_LEN);
        
        return 1;
    }
    else 
    {
        dac_data_0.setting.dac_setting = 0x00000; //set to zero
        printf("setting to 0\n");
        selected_dac = 0;
        return 1;
    }
#endif

    if (!generate_parity(in_command_buf)) {
        //if the incoming data is messy
        //assert error pin?
        return 0;//if checksum isn't valid, break it out.
    }

    //what type is it? we can use first few bits to define this. make some unions
    //where does it go?
    switch((in_command_buf[0] >> 3) & 0x0F) { //gets the 4 bits which define our address
        case 0x00://DAC0 SET, time to do all the important stuff
            memcpy(sys_analog_set.frame_buf, in_command_buf, BUF_LEN);
            dac_data_0.setting.dac_setting = sys_analog_set.setting.microvolts * dac0_scale;
            selected_dac = 0;
            return 1;

        case 0x01: //DAC1 SET
            memcpy(sys_analog_set.frame_buf, in_command_buf, BUF_LEN);
            dac_data_1.setting.dac_setting = sys_analog_set.setting.microvolts * dac1_scale;
            selected_dac = 1;
            return 1;

        case 0x02: //RESET, deal later
            return 1;//need command

        case 0x03://ADC0 GET, time to pack up the frame
            sys_analog_get.setting.microvolts = last_adc0 * adc0_scale;
            sys_analog_get.setting.checksum = 0;
            sys_analog_get.setting.checksum = generate_parity(sys_analog_get.frame_buf);
            memcpy(out_sys_buf, sys_analog_get.frame_buf, BUF_LEN);
            //checksum
            reply_pending = 1;
            return 0;//no command

        case 0x04://ADC1 GET
            sys_analog_get.setting.microvolts = last_adc1 * adc1_scale;
            memcpy(out_sys_buf, sys_analog_get.frame_buf, BUF_LEN);
            //checksum too
            reply_pending = 1;
            return 0;//no command

        default:
            return 0;
    }
}

bool generate_parity(uint8_t frame[DAC_LEN]){
    // parity with last bit
    uint32_t fullframe = (frame[0]) + (frame[1] << 8) + (frame [2] << 16) + (frame [3] << 24);
    return __builtin_parity(fullframe); // 1 if odd, 0 if even.
}

uint32_t readADC(uint8_t id)
{
    switch(id) {
        case 0:
            adc_select_input(DAC0_READADC);
            return adc_read();
        case 1:
            adc_select_input(DAC1_READADC);
            return adc_read();
        default:
            return 0;
    }
}

void dacInit(void)
{
    // DACs connected to SPI1
    gpio_init(LDAC);
    gpio_init(ALARM_DAC0);
    gpio_init(ALARM_DAC1);
    gpio_set_dir(LDAC, GPIO_OUT);
    gpio_set_dir(ALARM_DAC0, GPIO_IN);
    gpio_set_dir(ALARM_DAC1, GPIO_IN);

    gpio_put(LDAC, 1); // set high, active low

    // initialize data frame
    dac_data_0.setting.rw = 0; // 0 to write
    dac_data_1.setting.rw = 0;
    dac_data_0.setting.address = 0x01;
    dac_data_1.setting.address = 0x01;
    dac_data_0.setting.dac_setting = 0; // start at 0
    dac_data_1.setting.dac_setting = 0;

    //initialize config frames, all values default to zero
    dac_config1_0.setting.rw = 0;//0 to write
    dac_config1_1.setting.rw = 0;
    dac_config1_0.setting.address = 0x02;
    dac_config1_1.setting.address = 0x02;
    dac_config1_0.setting.en_tmp_cal = 1; // make temperature calibration available
    dac_config1_1.setting.en_tmp_cal = 1;
    dac_config1_0.setting.tnh_mask = 00; // default mask for track-and-hold deglitcher
    dac_config1_1.setting.tnh_mask = 00;
    dac_config1_0.setting.ldac_mode = 0; // use the LDAC input to trigger change
    dac_config1_1.setting.ldac_mode = 0; // but don't do it for testing!
    dac_config1_0.setting.fsdo = 0; // don't use "fast SDO"
    dac_config1_1.setting.fsdo = 0;
    dac_config1_0.setting.enalmp = 1; // make alarm pin functional
    dac_config1_1.setting.enalmp = 1;
    dac_config1_0.setting.dsdo = 1; // SDO enabled
    dac_config1_1.setting.dsdo = 1;
    dac_config1_0.setting.fset = 1; // fast settling
    dac_config1_1.setting.fset = 1;
    dac_config1_0.setting.vrefval = 0x03; // 0011b for 8.192v (7.5 +-1.25)
    dac_config1_1.setting.vrefval = 0x03;
    dac_config1_0.setting.pdn = 0; // don't power down the DAC
    dac_config1_1.setting.pdn = 0;

    // initialize trigger frame
    dac_trigger_0.setting.rw = 0; // 0 to write
    dac_trigger_0.setting.rw = 0;
    dac_trigger_0.setting.address = 0x04;
    dac_trigger_1.setting.address = 0x04;
    dac_trigger_0.setting.rcltemp = 0; // goes to 1 to initialize a temp calibration
    dac_trigger_1.setting.rcltemp = 0;
    dac_trigger_0.setting.srst = 0;
    dac_trigger_1.setting.srst = 0;
    dac_trigger_0.setting.sclr = 0;
    dac_trigger_1.setting.sclr = 0;

    //now we start writing
    gpio_put(CS_DAC0, 0); // enable both!
    gpio_put(CS_DAC1, 0); // config matches anyways so no issue with this.
    printf("Config struct results: ");
    printbuf(dac_config1_0.frame_buf, BUF_LEN);

    //spi_write_blocking(spi1, dac_config1_0.frame_buf, DAC_LEN); //write the base config
    spi_write_blocking(spi1, config_frame_buf, DAC_LEN); // write the base config
    printf("Desired results: ");
    printbuf(config_frame_buf, BUF_LEN);


    gpio_put(CS_DAC0, 1); // disable both again
    gpio_put(CS_DAC1, 1);
    printf("DACs enabled");
}

void spiInit(void)
{
    // set up CS pins, active low, default to high
    gpio_init(CS_DAC0);
    gpio_init(CS_DAC1);
    gpio_set_dir(CS_DAC0, GPIO_OUT);
    gpio_set_dir(CS_DAC1, GPIO_OUT);
    gpio_put(CS_DAC0, 1);
    gpio_put(CS_DAC1, 1);

    printf("Initializing SPI\n");
    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi0, 1000 * 1000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); // configure SPH = 1 for proper behavior with DAC
    spi_set_slave(spi0, true);
    gpio_set_function(4, GPIO_FUNC_SPI); // RX
    gpio_set_function(6, GPIO_FUNC_SPI); // SCK
    gpio_set_function(7, GPIO_FUNC_SPI); // TX
    gpio_set_function(5, GPIO_FUNC_SPI); // CSN
    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(4, 7, 6, 5, GPIO_FUNC_SPI));


    // creating an interrupt on SPI0 buffer filling up
    irq_add_shared_handler(SPI0_IRQ, onSpiInt, 1); 
    irq_set_enabled(SPI0_IRQ, true);
    // enabling the masking for the RX int
    *((io_rw_32 *) (SPI0_BASE + SPI_SSPIMSC_OFFSET))=(1<<2);

    // now we set up SPI1 as master.
    spi_init(spi1, 1000 * 1000);

    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); // configure SPH = 1 for proper behavior with DAC
    gpio_set_function(8,  GPIO_FUNC_SPI); // RX, 16
    gpio_set_function(10, GPIO_FUNC_SPI); // SCK, 18
    gpio_set_function(11, GPIO_FUNC_SPI); // TX, 19
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(8, 11, 10, GPIO_FUNC_SPI));
    
}

// keeping this for debug purposes, just prints out the supplied buffer
void printbuf(uint8_t *buf, size_t len)
{
    int i;
    for (i = 0; i < len; ++i) {
        printf("%02X", buf[i]);
        if (i % 16 == 15)
            putchar('\n');
        else
            putchar(' ');
    }
    // append trailing newline if there isn't one
    if (i % 16) {
        putchar('\n');
    }
}
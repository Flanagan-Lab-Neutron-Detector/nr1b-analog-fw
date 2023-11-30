/** @file spi_switch.c
 * @brief Main file for analog controller
 * 
 * @authors Isaac Kelly, Aidan Medcalf
 */

// stdlib
#include <stdio.h>
#include <string.h>

// pico libraries
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/adc.h"

// application libraries
#include "pindefs.h"

#define BASICDAC //test option; receiving any SPI transfer will give the DAC a setting
#undef BASICDAC

// Define SPI1_USE_HARDWARE to use the peripheral, otherwise SPI1 will be bit-banged
//#define SPI1_USE_HARDWARE
#undef SPI1_USE_HARDWARE

// Define SPI1_VERIFY_WRITES to read-back writed to the DAC
#undef SPI1_VERIFY_WRITES

// Initializing buffers
#define BUF_LEN         0x04 // 4 bytes to trigger interrupt
#define EMPTY_LEN       0x01 // single byte to clear buffer
#define DAC_LEN         0x04 // always 4 bytes, 32 bits

uint8_t in_sys_buf[BUF_LEN], out_sys_buf[BUF_LEN], in_dac_buf[BUF_LEN], out_dac_buf[BUF_LEN], in_command_buf[BUF_LEN];
uint8_t empty_buf, null_buf[EMPTY_LEN];//for flushing buffers

// Controller Message frames

/** @brief Analog set frame
 * Controller SPI frame to set DAC output.
 */
struct analog_set_frame {
    uint32_t rw         : 1;  // Read/Write bit
    uint32_t address    : 4;  // SPI address (command)
    uint32_t microvolts : 23; // DAC output in raw counts
    uint32_t reserved   : 3;  // Unused (zero)
    uint32_t checksum   : 1;  // Parity bit
};

/** @brief Reset frame
 * Controller SPI frame to reset DACs and MCU.
 */
struct reset_frame {
    bool rw             : 1;  // Read/Write bit
    uint8_t address     : 4;  // SPI address (command)
    bool mcu            : 1;  // Reset MCU?
    bool dac0           : 1;  // Reset DAC0?
    bool dac1           : 1;  // Reset DAC1?
    uint32_t reserved   : 23; // Unused (zero)
    bool checksum       : 1;  // Parity bit
};

/** @brief Analog get frame
 * Controller SPI frame to get ADC input.
 */
struct analog_get_frame {
    bool rw             : 1;  // Read/Write bit
    uint8_t address     : 4;  // SPI address (command)
    uint32_t microvolts : 23; // ADC input in raw counts
    uint8_t reserved    : 3;  // Unused (zero)
    bool checksum       : 1;  // Parity bit
};

// DAC Message frames

/** @brief DAC data frame
 * DAC SPI frame to set DAC output.
 * @note DAC SPI frames are reversed.
 */
struct dac_data_frame {
    uint8_t reserved0    : 4;  // Unused (zero)
    uint32_t dac_setting : 20; // DAC output in raw counts
    uint8_t address      : 7;  // Data address
    bool rw              : 1;  // Read/Write bit
};

/** @brief DAC config1 frame
 * DAC SPI frame to set DAC configuration.
 * @note DAC SPI frames are reversed.
 */
struct dac_config1_frame {
    uint32_t reserved3  : 4;
    uint32_t pdn        : 1;
    uint32_t reserved2  : 1;
    uint32_t vrefval    : 4;
    uint32_t fset       : 1;
    uint32_t dsdo       : 1;
    uint32_t enalmp     : 1;
    uint32_t fsdo       : 1;
    uint32_t ldac_mode  : 1;
    uint32_t reserved1  : 3;
    uint32_t tnh_mask   : 2;
    uint32_t reserved0  : 3;
    uint32_t en_tmp_cal : 1;
    uint32_t address    : 7;
    uint32_t rw         : 1;
};

/** @brief DAC trigger frame
 * DAC SPI frame to set DAC trigger configuration.
 * @note DAC SPI frames are reversed.
 */
struct dac_trigger_frame {
    bool rw              : 1;
    uint8_t address      : 7;
    uint16_t reserved0   : 15;
    bool rcltemp         : 1;
    bool reserved1       : 1;
    bool srst            : 1;
    bool sclr            : 1;
    bool reserved2       : 1;
    uint8_t reserved3    : 4;
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

// Pre-set buffers

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

// Comms globals

union dac_data dac_data_0, dac_data_1, dac_data_verify, dac_data_read;
union dac_config1 dac_config1_0, dac_config1_1, dac_config1_0_verify1, dac_config1_0_verify2;
union dac_trigger dac_trigger_0, dac_trigger_1;

union analog_set_data sys_analog_set;
union reset_data sys_reset;
union analog_get_data sys_analog_get;

// Main loop globals

bool command_received = false;
bool reply_pending = false;
bool command_pending = false;
uint8_t selected_dac = 0;
uint32_t raw_voltage_in; // we'll convert this to a DAC setting


bool parseCommand(void);
bool generate_parity(uint8_t frame[DAC_LEN]);
void spiInit(void);
void printbuf(uint8_t *buf, size_t len);
bool dacInit(void);

uint32_t readADC(uint8_t id);

uint32_t last_adc0 = 24; // test
uint32_t last_adc1 = 56; // test

/** @brief SPI interrupt handler
 * ISR from SSPXINTR. Reads SPI command from controller and sets @ref command_received flag.
*/
void onSpiInt(void)
{
    // Since we have at least 4 bytes in our buffer (interrupt has fired) we read 4.
    spi_read_blocking(spi0, empty_buf, in_sys_buf, BUF_LEN); // Read buffer in, send 0s back.
    // Now we clear the rest of the buffer.
    while (spi_is_readable(spi0)) {
        spi_read_blocking(spi0, empty_buf, null_buf, EMPTY_LEN); // Read buffer into nowhere, send 0s back.
    }
    command_received = true; // Set the flag, will be processed by main loop.
}

int hwspi1_write_blocking(const uint8_t *src, size_t len)
{
    return spi_write_blocking(spi1, src, len);
}

int swspi1_write_blocking(const uint8_t *src, size_t len)
{
    // initial state
    // for each byte:
    //   for each bit:
    //     set MOSI
    //     clock low?
    //     wait
    //     clock high?
    //     wait

    for (int i = 0; i < len; i++) {
        uint8_t byte = src[i];
        for (int j = 0; j < 8; j++) {
            gpio_put(SPI1_TX, byte & 0x80);
            byte <<= 1;
            gpio_put(SPI1_SCK, 1);
            busy_wait_us(1);
            gpio_put(SPI1_SCK, 0);
            busy_wait_us(1);
        }
    }

    return len;
}

int swspi1_readwrite_blocking(const uint8_t *src, uint8_t *dst, size_t length)
{
    // like swspi1_write_blocking, but on each falling edge of SPI1_SCK, read from SPI1_RX

    for (int i = 0; i < length; i++) {
        uint8_t byte = src[i];
        // dst[i] is 8 bits and we shift in 8 bits, so no need to clear
        for (int j = 0; j < 8; j++) {
            gpio_put(SPI1_TX, byte & 0x80);
            byte <<= 1;
            gpio_put(SPI1_SCK, 1);
            busy_wait_us(1);
            dst[i] <<= 1;
            dst[i] |= (((uint)gpio_get(SPI1_RX)) & 0x01);
            gpio_put(SPI1_SCK, 0);
            busy_wait_us(1);
        }
    }

    return length;
}

/*
int dac_send(uint cspin, union dac_data *data)
{
    gpio_put(cspin, 0); // select DAC
    // send in reverse order
    for(int i = 0; i < BUF_LEN; i++) {
        data_frame_buf[i] = data->frame_buf[(BUF_LEN - 1) - i];
    }
    //gpio_put(cspin, 0); // select DAC
#ifdef SPI1_USE_HARDWARE
    int nwritten = hwspi1_write_blocking(spi1, data_frame_buf, DAC_LEN); // write the base config
#else
    int nwritten = swspi1_write_blocking(data_frame_buf, DAC_LEN); // write the base config
#endif
    gpio_put(cspin, 1); // deselect DAC
    return nwritten;
}
*/

int dac_send(uint cspin, union dac_data *data)
{
    gpio_put(cspin, 0); // select DAC
    // send in reverse order
    for(int i = 0; i < BUF_LEN; i++) {
        data_frame_buf[i] = data->frame_buf[(BUF_LEN - 1) - i];
    }
    //gpio_put(cspin, 0); // select DAC
#ifdef SPI1_USE_HARDWARE
    int nwritten = hwspi1_write_blocking(spi1, data_frame_buf, DAC_LEN); // write the base config
#else
    int nwritten = swspi1_write_blocking(data_frame_buf, DAC_LEN); // write the base config
#endif
    gpio_put(cspin, 1); // deselect DAC

#ifdef SPI1_VERIFY_WRITES
    printf("Wrote: ");
    printbuf(data_frame_buf, DAC_LEN);

    // read back
    gpio_put(cspin, 0); // select DAC
    // send in reverse order
    for(int i = 0; i < BUF_LEN; i++) {
        data_frame_buf[i] = dac_data_read.frame_buf[(BUF_LEN - 1) - i];
    }
    //gpio_put(cspin, 0); // select DAC
#ifdef SPI1_USE_HARDWARE
    hwspi1_readwrite_blocking(spi1, data_frame_buf, DAC_LEN); // write the base config
#else
    swspi1_readwrite_blocking(data_frame_buf, dac_data_verify.frame_buf, DAC_LEN); // write the base config
#endif
    gpio_put(cspin, 1); // deselect DAC

    printf("Wrote: ");
    printbuf(data_frame_buf, DAC_LEN);
    printf("Read:  ");
    printbuf(dac_data_verify.frame_buf, DAC_LEN);
#endif

    return nwritten;
}

int main(void)
{
    stdio_init_all(); // for printf

    sleep_ms(3000);

    adc_init();
    //set up LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    // test points
    gpio_init(TP1);
    gpio_init(TP2);
    gpio_init(TP3);
    gpio_init(TP4);
    gpio_set_dir(TP1, GPIO_OUT);
    gpio_set_dir(TP2, GPIO_OUT);
    gpio_set_dir(TP3, GPIO_OUT);
    gpio_set_dir(TP4, GPIO_OUT);
    gpio_put(TP1, 0);
    gpio_put(TP2, 0);
    gpio_put(TP3, 0);
    gpio_put(TP4, 0);

    //status indication
    gpio_init(ACK);
    gpio_init(ERROR);
    gpio_set_dir(ACK, GPIO_OUT);
    gpio_set_dir(ERROR, GPIO_OUT);
    adc_gpio_init(DAC0_READPIN);
    adc_gpio_init(DAC1_READPIN);
    // ACK and ERROR initially low
    gpio_put(ACK, 0);
    gpio_put(ERROR, 0);

    spiInit(); //set up SPI peripherals and CS pins

    bool dac_init_success = dacInit();
    if (!dac_init_success)
        gpio_put(ERROR, 1);
    printf("dac_init_success = %d\n", dac_init_success);

    puts("goliath online");

    while (1) //main loop
    {
        // indicate that we are ready to receive a command
        gpio_put(ACK, 0);

        // Service flags

        if (command_received) { // we have a command to process
            memcpy(in_command_buf, in_sys_buf, BUF_LEN); // in_sys_buf is written to by ISR
            command_received = false; // clear flag

            // parse command, verify, interpret, choose next step.
            command_pending = parseCommand();
        }

        if (reply_pending) { // need to send data back to controller
            spi_write_read_blocking(spi0, out_sys_buf, in_sys_buf, BUF_LEN); // write DAC response, get new command
        }

        /*command_pending = true;
        selected_dac = selected_dac ? 0 : 1; // toggle DAC selection
        switch (selected_dac) {
            case 0: // DAC0
                dac_data_0.setting.dac_setting = dac_data_0.setting.dac_setting + 100000 % (1 << 20);
                break;
            case 1: // DAC1
                dac_data_1.setting.dac_setting = dac_data_1.setting.dac_setting + 100000 % (1 << 20);
                break;
            default:
                break;
        }*/

        if (command_pending) { // we need to send something out to DAC
            int bytes_sent = 0;
            switch (selected_dac) {
                case 0: // DAC0
                    gpio_put(ACK, 1); // Acknowledge command
                    puts("sending to DAC0");
                    //gpio_put(CS_DAC0, 0); // select DAC0
                    bytes_sent = dac_send(CS_DAC0, &dac_data_0);
                    //gpio_put(CS_DAC0, 1); // deselect DAC0
                    break;
                case 1: // DAC1
                    gpio_put(ACK, 1); // Acknowledge command
                    puts("sending to DAC1");
                    //gpio_put(CS_DAC1, 0); // select DAC1
                    bytes_sent = dac_send(CS_DAC1, &dac_data_1);
                    //gpio_put(CS_DAC1, 1); // deselect DAC1
                    break;
                default:
                    break;
            }
            if (bytes_sent != DAC_LEN) {
                gpio_put(ERROR, 1); // indicate error
            }
            command_pending = false; // clear flag
        }

        sleep_ms(100);
    }

    return 0;
}

/** @brief Parse command from controller
 * @return true if main loop has a command to process
 */
bool parseCommand(void)
{
    //printf("parsing command \n");
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

    // Check parity
    if (generate_parity(in_command_buf)) {
        // TODO: assert error pin
        printf("Bad parity: %d\n", generate_parity(in_command_buf));
        printbuf(in_command_buf, BUF_LEN);
        return false; //if checksum isn't valid, break it out.
    }

    // Decode address (command)
    switch((in_command_buf[0] >> 1) & 0x0F) { //gets the 4 bits which define our address
        case 0x00:// DAC0 SET
            memcpy(sys_analog_set.frame_buf, in_command_buf, BUF_LEN);
            dac_data_0.setting.dac_setting = sys_analog_set.setting.microvolts;
            //printf("dac_data_0 setting = %d*%.1f = %d\n", sys_analog_set.setting.microvolts, dac0_scale, dac_data_0.setting.dac_setting);
            selected_dac = 0;
            return true; // we have a command to process

        case 0x01: // DAC1 SET
            memcpy(sys_analog_set.frame_buf, in_command_buf, BUF_LEN);
            dac_data_1.setting.dac_setting = sys_analog_set.setting.microvolts;
            //printf("dac_data_1 setting = %d*%.1f = %d\n", sys_analog_set.setting.microvolts, dac1_scale, dac_data_1.setting.dac_setting);
            selected_dac = 1;
            return true; // we have a command to process

        case 0x02: // RESET TODO
            return true; // we have a command to process

        case 0x03: // ADC0 GET
            sys_analog_get.setting.microvolts = last_adc0;
            sys_analog_get.setting.checksum = 0;
            sys_analog_get.setting.checksum = generate_parity(sys_analog_get.frame_buf);
            memcpy(out_sys_buf, sys_analog_get.frame_buf, BUF_LEN);
            reply_pending = 1;
            return false; // no command

        case 0x04: // ADC1 GET
            sys_analog_get.setting.microvolts = last_adc1;
            memcpy(out_sys_buf, sys_analog_get.frame_buf, BUF_LEN);
            reply_pending = 1;
            return false; // no command

        default:
            return false; // no command
    }
}

/** @brief Generate parity bit for a frame
 * @param frame frame to generate parity for
 * @return parity bit
 */
bool generate_parity(uint8_t frame[DAC_LEN])
{
    // parity with last bit
    uint32_t fullframe = (frame[0]) + (frame[1] << 8) + (frame [2] << 16) + (frame [3] << 24);
    return __builtin_parity(fullframe); // 1 if odd, 0 if even.
}

/** @brief Read ADC
 * @param id ADC to read (0 or 1)
 * @return ADC value or 0 if invalid id
 */
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

void printbin(char *pref, uint8_t *data, uint32_t len)
{
    if (pref != NULL)
        printf("  % 10s", pref);
    for (int bi = 0; bi < len; bi++) {
        printf("  %0d%0d%0d%0d %0d%0d%0d%0d",
               (data[bi] & 0x80) >> 7,
               (data[bi] & 0x40) >> 6,
               (data[bi] & 0x20) >> 5,
               (data[bi] & 0x10) >> 4,
               (data[bi] & 0x08) >> 3,
               (data[bi] & 0x04) >> 2,
               (data[bi] & 0x02) >> 1,
               (data[bi] & 0x01) >> 0);
    }
    putchar('\n');
}

/** @brief Initialize DACs
 * @return true if successful
 */
bool dacInit(void)
{
    // DACs connected to SPI1
    gpio_init(LDAC);
    gpio_init(ALARM_DAC0);
    gpio_init(ALARM_DAC1);
    gpio_set_dir(LDAC, GPIO_OUT);
    gpio_set_dir(ALARM_DAC0, GPIO_IN);
    gpio_set_dir(ALARM_DAC1, GPIO_IN);

    gpio_put(LDAC, 1); // set high, active low

    for (int bi = 0; bi < 4; bi ++) {
        printf("  %0d%0d%0d%0d %0d%0d%0d%0d",
               (dac_config1_0.frame_buf[bi] & 0x80) >> 7,
               (dac_config1_0.frame_buf[bi] & 0x40) >> 6,
               (dac_config1_0.frame_buf[bi] & 0x20) >> 5,
               (dac_config1_0.frame_buf[bi] & 0x10) >> 4,
               (dac_config1_0.frame_buf[bi] & 0x08) >> 3,
               (dac_config1_0.frame_buf[bi] & 0x04) >> 2,
               (dac_config1_0.frame_buf[bi] & 0x02) >> 1,
               (dac_config1_0.frame_buf[bi] & 0x01) >> 0);
    }
    putchar('\n');

    // clear dac_config1_0 and dac_config1_0_verify
    for (int i = 0; i < BUF_LEN; i++) {
        dac_config1_0.frame_buf[i] = 0;
    }
    // for each bitfield, set field to all ones, print, set to zero
#define SETPRINT(F, V) dac_config1_0.setting.F = V; printbin(#F, dac_config1_0.frame_buf, BUF_LEN); dac_config1_0.setting.F = 0;
    SETPRINT(rw, 1);
    SETPRINT(address, 0x7F);
    SETPRINT(en_tmp_cal, 1);
    SETPRINT(reserved0, 0x7);
    SETPRINT(tnh_mask, 0x3);
    SETPRINT(reserved1, 0x7);
    SETPRINT(ldac_mode, 1);
    SETPRINT(fsdo, 1);
    SETPRINT(enalmp, 1);
    SETPRINT(dsdo, 1);
    SETPRINT(fset, 1);
    SETPRINT(vrefval, 0xF);
    SETPRINT(reserved2, 1);
    SETPRINT(pdn, 1);
    SETPRINT(reserved3, 0xF);
#undef SETPRINT

    /*
    struct dac_config1_frame {
    bool rw              : 1;
    uint8_t address      : 7;
    bool en_tmp_cal      : 1;
    uint8_t reserved0    : 3;
    uint8_t tnh_mask     : 2;
    uint8_t reserved1    : 3;
    bool ldac_mode       : 1;
    bool fsdo            : 1;
    bool enalmp          : 1;
    bool dsdo            : 1;
    bool fset            : 1;
    uint8_t vrefval      : 4;
    bool reserved2       : 1;
    bool pdn             : 1;
    uint8_t reserved3    : 4;
};*/

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
    dac_config1_0.setting.en_tmp_cal = 0; // do not make temperature calibration available
    dac_config1_1.setting.en_tmp_cal = 0;
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
    dac_trigger_1.setting.rw = 0;
    dac_trigger_0.setting.address = 0x04;
    dac_trigger_1.setting.address = 0x04;
    dac_trigger_0.setting.rcltemp = 0; // goes to 1 to initialize a temp calibration
    dac_trigger_1.setting.rcltemp = 0;
    dac_trigger_0.setting.srst = 0;
    dac_trigger_1.setting.srst = 0;
    dac_trigger_0.setting.sclr = 0;
    dac_trigger_1.setting.sclr = 0;

    printf("Config struct results: ");
    printbuf(dac_config1_0.frame_buf, BUF_LEN);
    printf("Desired results: ");
    printbuf(config_frame_buf, BUF_LEN);

    printf("%zu =? %zu\n", sizeof(struct dac_config1_frame), sizeof(union dac_config1));

    printf("  rw         = %0X\n", dac_config1_0.setting.rw);
    printf("  address    = %0X\n", dac_config1_0.setting.address);
    printf("  en_tmp_cal = %0X\n", dac_config1_0.setting.en_tmp_cal);
    printf("  reserved0  = %0X\n", dac_config1_0.setting.reserved0);
    printf("  tnh_mask   = %0X\n", dac_config1_0.setting.tnh_mask);
    printf("  reserved1  = %0X\n", dac_config1_0.setting.reserved1);
    printf("  ldac_mode  = %0X\n", dac_config1_0.setting.ldac_mode);
    printf("  fsdo       = %0X\n", dac_config1_0.setting.fsdo);
    printf("  enalmp     = %0X\n", dac_config1_0.setting.enalmp);
    printf("  dsdo       = %0X\n", dac_config1_0.setting.dsdo);
    printf("  fset       = %0X\n", dac_config1_0.setting.fset);
    printf("  vrefval    = %0X\n", dac_config1_0.setting.vrefval);
    printf("  reserved2  = %0X\n", dac_config1_0.setting.reserved2);
    printf("  pdn        = %0X\n", dac_config1_0.setting.pdn);
    printf("  reserved3  = %0X\n", dac_config1_0.setting.reserved3);

    //now we start writing
    gpio_put(CS_DAC0, 0); // enable both!
    gpio_put(CS_DAC1, 0); // config matches anyways so no issue with this.

    //spi_write_blocking(spi1, dac_config1_0.frame_buf, DAC_LEN); //write the base config
    //int bytes_written = spi_write_blocking(spi1, config_frame_buf, DAC_LEN); // write the base config
    // rotate bytes
    for (int i = 0; i < BUF_LEN; i++) {
        config_frame_buf[i] = dac_config1_0.frame_buf[DAC_LEN - i - 1];
    }
    printf("Wrote: ");
    printbuf(config_frame_buf, BUF_LEN);
#ifdef SPI1_USE_HARDWARE
    int bytes_written = hwspi1_write_blocking(spi1, config_frame_buf, DAC_LEN); // write the base config
#else
    int bytes_written = swspi1_write_blocking(config_frame_buf, DAC_LEN); // write the base config
#endif


    gpio_put(CS_DAC0, 1); // disable both again
    gpio_put(CS_DAC1, 1);
    puts("DACs enabled");

    if (bytes_written != DAC_LEN) {
        printf("Error: %d bytes written, expected %d\n", bytes_written, DAC_LEN);
        return false;
    }

    // now read back the config

    // read into dac_config1_0_verify, which is already zeroed
    // use dac_config1_0 for read command (set rw)
    for (int i = 0; i < BUF_LEN; i++) {
        dac_config1_0.frame_buf[i] = 0;
        dac_config1_0_verify1.frame_buf[i] = 0;
        dac_config1_0_verify2.frame_buf[i] = 0;
    }
    dac_config1_0.setting.rw = 1; // set read bit
    dac_config1_0.setting.address = 0x02;
    // rotate bytes
    for (int i = 0; i < BUF_LEN; i++) {
        config_frame_buf[i] = dac_config1_0.frame_buf[DAC_LEN - i - 1];
    }
    printf("Wrote: ");
    printbuf(config_frame_buf, BUF_LEN);

    // signal to LA
    gpio_put(TP1, 1);
    gpio_put(TP2, 1);

    gpio_put(CS_DAC0, 0); // enable
#ifdef SPI1_USE_HARDWARE
    hwspi1_readwrite_blocking(spi1, config_frame_buf, dac_config1_0_verify1.frame_buf, DAC_LEN);
#else
    swspi1_readwrite_blocking(config_frame_buf, dac_config1_0_verify1.frame_buf, DAC_LEN);
#endif
    gpio_put(CS_DAC0, 1); // disable

    // done with first word
    gpio_put(TP2, 0);

    // send nop and read
    for (int i = 0; i < BUF_LEN; i++) {
        dac_config1_0.frame_buf[i] = 0;
    }
    // nop
    dac_config1_0.setting.rw = 0; // set read bit
    dac_config1_0.setting.address = 0x00;
    // rotate bytes
    for (int i = 0; i < BUF_LEN; i++) {
        config_frame_buf[i] = dac_config1_0.frame_buf[DAC_LEN - i - 1];
    }
    printf("Wrote: ");
    printbuf(config_frame_buf, BUF_LEN);

    // second word
    gpio_put(TP3, 1);

    gpio_put(CS_DAC0, 0); // enable
#ifdef SPI1_USE_HARDWARE
    hwspi1_readwrite_blocking(spi1, config_frame_buf, dac_config1_0_verify2.frame_buf, DAC_LEN);
#else
    swspi1_readwrite_blocking(config_frame_buf, dac_config1_0_verify2.frame_buf, DAC_LEN);
#endif
    gpio_put(CS_DAC0, 1); // disable

    // done
    gpio_put(TP3, 0);
    gpio_put(TP1, 0);

    printf("Read: ");
    printbuf(dac_config1_0_verify1.frame_buf, BUF_LEN);
    printf("      ");
    printbuf(dac_config1_0_verify2.frame_buf, BUF_LEN);

    // now set to zero
    // dac_data_x set above
    // DAC 0
    bytes_written = dac_send(CS_DAC0, &dac_data_0);
    if (bytes_written != DAC_LEN) {
        printf("Error: %d bytes written, expected %d\n", bytes_written, DAC_LEN);
        return false;
    }
    // DAC 1
    bytes_written = dac_send(CS_DAC1, &dac_data_1);
    if (bytes_written != DAC_LEN) {
        printf("Error: %d bytes written, expected %d\n", bytes_written, DAC_LEN);
        return false;
    }

    // set up verification structures

    // clear
    for (uint32_t i=0; i<DAC_LEN; i++) {
        dac_data_verify.frame_buf[i] = 0;
        dac_data_read.frame_buf[i] = 0;
    }
    // set up read
    dac_data_read.setting.rw = 1;
    dac_data_read.setting.address = 0x01;

    return true;
}

/** @brief Initialize System SPI
 */
void spiSysInit(void)
{
    // Enable SPI 0 at 4 MHz and connect to GPIOs
    uint spi0_baud = 1 * 1000 * 1000;
    spi_init(spi0, spi0_baud);
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
}

/** @brief Initialize DAC SPI (HW driver)
 */
void spiDacHwInit(void)
{
    // now we set up SPI1 as master.
    uint spi1_baud = 1 * 1000 * 100;
    spi_init(spi1, spi1_baud);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); // configure SPH = 1 for proper behavior with DAC
    //hw_write_masked(&spi_get_hw(spi1)->cr0, ((uint)1 << SPI_SSPCR0_FRF_LSB), SPI_SSPCR0_FRF_BITS); // TI frame format
    gpio_set_function(8,  GPIO_FUNC_SPI); // RX, 16
    gpio_set_function(10, GPIO_FUNC_SPI); // SCK, 18
    gpio_set_function(11, GPIO_FUNC_SPI); // TX, 19
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(8, 11, 10, GPIO_FUNC_SPI));
}

/** @brief Initialize DAC SPI (SW driver)
 */
void spiDacSwInit(void)
{
    gpio_set_function(SPI1_RX,  GPIO_FUNC_NULL); // RX, 16
    gpio_set_function(SPI1_SCK, GPIO_FUNC_NULL); // SCK, 18
    gpio_set_function(11, GPIO_FUNC_NULL); // TX, 19

    gpio_init(SPI1_RX); // RX, 16
    gpio_init(SPI1_SCK); // SCK, 18
    gpio_init(SPI1_TX); // TX, 19

    gpio_set_dir(SPI1_RX, GPIO_IN);
    gpio_set_dir(SPI1_SCK, GPIO_OUT);
    gpio_set_dir(SPI1_TX, GPIO_OUT);

    gpio_put(SPI1_SCK, 0);
    gpio_put(SPI1_TX, 0);
}

/** @brief Initialize SPI
 */
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
    spiSysInit();

#ifdef SPI1_USE_HARDWARE
    spiDacHwInit();
#else
    spiDacSwInit();
#endif
}

/** @brief Print a buffer in hex (debug)
 * @param buf buffer to print
 * @param len length of buffer
 */
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
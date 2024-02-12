/** @file main.c
 * @brief Main file for analog controller
 * 
 * @authors Isaac Kelly, Aidan Medcalf
 */

/* Todo
 * - Get ADC working
 * - Figure out how to provide SPI data (for ADC reporting), or switch to bit-banging, or switch to something else
 * - Move defines to CMAKE
 * - Fix message structs
 * - Report errors with ERR pin
 * - Add watchdog
 * - Implement reset command
 * - Split into multiple files
 * - Guard debug printing with a define
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

// Define SPI1_USE_HARDWARE to use the peripheral, otherwise SPI1 will be bit-banged
//#define SPI1_USE_HARDWARE
#undef SPI1_USE_HARDWARE

// Define SPI1_VERIFY_WRITES to read-back writed to the DAC
#undef SPI1_VERIFY_WRITES

// Initializing buffers
#define BUF_LEN         0x04 // 4 bytes to trigger interrupt
#define EMPTY_LEN       0x01 // single byte to clear buffer
#define DAC_LEN         0x04 // always 4 bytes, 32 bits

uint8_t in_sys_buf[BUF_LEN], out_sys_buf[BUF_LEN], in_dac_buf[BUF_LEN], out_dac_buf[BUF_LEN];
//uint8_t in_command_buf[BUF_LEN];
uint8_t empty_buf, null_buf[EMPTY_LEN];//for flushing buffers

enum spi_command {
    CMD_DAC0_SET = 0x01,
    CMD_DAC1_SET = 0x02,
    CMD_RESET    = 0x03,
    CMD_ADC0_GET = 0x04,
    CMD_ADC1_GET = 0x05
};

// Controller Message frames

/** @brief Analog set frame
 * Controller SPI frame to set DAC output.
 */
struct analog_set_frame {
    uint32_t rw         : 1;  // Read/Write bit
    uint32_t address    : 4;  // SPI address (command)
    uint32_t dac_counts : 23; // DAC output in raw counts
    uint32_t reserved   : 3;  // Unused (zero)
    uint32_t checksum   : 1;  // Parity bit
};

/** @brief Reset frame
 * Controller SPI frame to reset DACs and MCU.
 */
struct reset_frame {
    uint32_t rw         : 1;  // Read/Write bit
    uint32_t address    : 4;  // SPI address (command)
    uint32_t mcu        : 1;  // Reset MCU?
    uint32_t dac0       : 1;  // Reset DAC0?
    uint32_t dac1       : 1;  // Reset DAC1?
    uint32_t reserved   : 23; // Unused (zero)
    uint32_t checksum   : 1;  // Parity bit
};

/** @brief Analog get frame
 * Controller SPI frame to get ADC input.
 */
struct analog_get_frame {
    uint32_t rw         : 1;  // Read/Write bit
    uint32_t address    : 4;  // SPI address (command)
    uint32_t adc_counts : 23; // ADC input in raw counts
    uint32_t reserved   : 3;  // Unused (zero)
    uint32_t checksum   : 1;  // Parity bit
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

union spi_command_data {
    struct analog_set_frame analog_set;
    struct analog_get_frame analog_get;
    struct reset_frame      reset;
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

union analog_get_data sys_analog_get;

// Main loop globals
volatile bool command_received = false;

void handleCommand(union spi_command_data *command_data);
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

/** @brief Write to SPI1 hardware peripheral (see @ref spi_write_blocking)
 * @param src data to write
 * @param len length of data
 * @return number of bytes written
 */
int hwspi1_write_blocking(const uint8_t *src, size_t len)
{
    return spi_write_blocking(spi1, src, len);
}

/** @brief@rite to SPI1 in software
 * @param src data to write
 * @param len length of data
 * @return number of bytes written
 */
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

/** @brief Read/write SPI1 in software
 * @param src buffer to write
 * @param dst buffer to read into
 * @param length length of data
 * @return number of bytes read
 */
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

/** @brief Write data to DACs
 * @param cspin selected DAC CS pin
 * @param data data to write
 * @return number of bytes written
 * @note If @ref SPI1_VERIFY_WRITES is defined, reads back data and prints to console
 */
int dac_write(uint cspin, union dac_data *data)
{
    gpio_put(cspin, 0); // select DAC
    // send in reverse order
    for(int i = 0; i < BUF_LEN; i++) {
        data_frame_buf[i] = data->frame_buf[(BUF_LEN - 1) - i];
    }
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

int dac_send(int dac, union dac_data *data)
{
    int bytes_sent = 0;
    switch (dac) {
        case 0:
            puts("sending to DAC0");
            bytes_sent = dac_write(CS_DAC0, data);
            gpio_put(ACK, 1); // Acknowledge command
            break;
        case 1:
            puts("sending to DAC1");
            bytes_sent = dac_write(CS_DAC1, data);
            gpio_put(ACK, 1); // Acknowledge command
            break;
        default:
            puts("invalid DAC");
            gpio_put(ERROR, 1); // indicate error
            break;
    }
    if (bytes_sent != DAC_LEN) {
        gpio_put(ERROR, 1); // indicate error
    }
    return bytes_sent;
}

int main(void)
{
    stdio_init_all(); // for printf

    sleep_ms(100);

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
            union spi_command_data in_command_data;
            memcpy(in_command_data.frame_buf, in_sys_buf, BUF_LEN); // in_sys_buf is written to by ISR
            command_received = false; // clear flag

            // parse command, verify, interpret, choose next step.
            handleCommand(&in_command_data);
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

        sleep_us(100);
    }

    return 0;
}

/** @brief Parse command from controller
 * @return true if main loop has a command to process
 */
void handleCommand(union spi_command_data *command_data)
{
    // Verify parity
    bool cmd_parity = generate_parity(command_data->frame_buf);
    if (cmd_parity) {
        printf("Bad parity: %d\n", cmd_parity);
        printbuf(command_data->frame_buf, BUF_LEN);
        gpio_put(ERROR, 1); // indicate error
    } else {
        // Decode address (command)
        switch((enum spi_command)((command_data->frame_buf[0] >> 1) & 0x0F)) { //gets the 4 bits which define our address
            case CMD_DAC0_SET:
                dac_data_0.setting.dac_setting = command_data->analog_set.dac_counts;
                //printf("dac_data_0 setting = %d*%.1f = %d\n", sys_analog_set.setting.dac_counts, dac0_scale, dac_data_0.setting.dac_setting);
                dac_send(0, &dac_data_0);
                break;
            case CMD_DAC1_SET:
                dac_data_1.setting.dac_setting = command_data->analog_set.dac_counts;
                //printf("dac_data_1 setting = %d*%.1f = %d\n", sys_analog_set.setting.dac_counts, dac1_scale, dac_data_1.setting.dac_setting);
                dac_send(1, &dac_data_1);
                break;
            case CMD_RESET:
                // TODO
                break;
            case CMD_ADC0_GET:
                sys_analog_get.setting.adc_counts = last_adc0;
                sys_analog_get.setting.checksum = 0;
                sys_analog_get.setting.checksum = generate_parity(sys_analog_get.frame_buf);
                memcpy(out_sys_buf, sys_analog_get.frame_buf, BUF_LEN);
                spi_write_read_blocking(spi0, out_sys_buf, in_sys_buf, BUF_LEN); // write DAC response, get new command
                break;
            case CMD_ADC1_GET:
                sys_analog_get.setting.adc_counts = last_adc1;
                sys_analog_get.setting.checksum = 0;
                sys_analog_get.setting.checksum = generate_parity(sys_analog_get.frame_buf);
                memcpy(out_sys_buf, sys_analog_get.frame_buf, BUF_LEN);
                spi_write_read_blocking(spi0, out_sys_buf, in_sys_buf, BUF_LEN); // write DAC response, get new command
                break;
            default:
                break;
        }
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
    bytes_written = dac_write(CS_DAC0, &dac_data_0);
    if (bytes_written != DAC_LEN) {
        printf("Error: %d bytes written, expected %d\n", bytes_written, DAC_LEN);
        return false;
    }
    // DAC 1
    bytes_written = dac_write(CS_DAC1, &dac_data_1);
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
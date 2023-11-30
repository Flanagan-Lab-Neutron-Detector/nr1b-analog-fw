/** @file pindefs.h
 * @brief Pin-related definitions
 * 
 * @authors Aidan Medcalf
 */

#ifndef PINDEFS_H
#define PINDEFS_H

// GPIO defines
#define CS_DAC0          9  // DAC0 Negative Chip Select
#define ALARM_DAC0       14 // DAC0 tempcal done signal, active low
#define CS_DAC1          12 // DAC1 Negative Chip Select
#define ALARM_DAC1       15 // DAC1 tempcal done signal, active low
#define LDAC             13 // register -> dac output, active low, enable with LDACMODE
#define ACK              2  // acknowledge to controller
#define ERROR            3  // error to controller
#define DAC0_READPIN     26 // DAC0 ADC GPIO pin
#define DAC0_READADC     0  // DAC0 ADC channel
#define DAC1_READPIN     27 // DAC1 ADC GPIO pin
#define DAC1_READADC     1  // DAC1 ADC channel
#define SPI1_TX          11 // SPI1 TX
#define SPI1_SCK         10 // SPI1 SCK
#define SPI1_RX          8  // SPI1 RX
#define TP1              16
#define TP2              17
#define TP3              22
#define TP4              28

#endif // PINDEFS_H

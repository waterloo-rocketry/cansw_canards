#ifndef ADS1219_REG_H
#define ADS1219_REG_H

/* Default I2C address, A0 and A1 both to DGND */
#define ADS1219_I2C_ADDRESS 0x40

/* Command bytes (table 7, p.25 in datasheet) */
#define ADS1219_CMD_RESET 0x06
#define ADS1219_CMD_START_SYNC 0x08
#define ADS1219_CMD_POWERDOWN 0x02
#define ADS1219_CMD_RDATA 0x10
#define ADS1219_CMD_RREG_CONFIG 0x20
#define ADS1219_CMD_RREG_STATUS 0x24
#define ADS1219_CMD_WREG 0x40

/* Config register field masks (1 = untouched, 0 = field bits) */
#define ADS1219_CONFIG_MASK_VREF 0xFE /* bit 0 */
#define ADS1219_CONFIG_MASK_CM 0xFD /* bit 1 */
#define ADS1219_CONFIG_MASK_DR 0xF3 /* bits 2-3 */
#define ADS1219_CONFIG_MASK_GAIN 0xEF /* bit 4 */
#define ADS1219_CONFIG_MASK_MUX 0x1F /* bits 5-7 */

/* MUX field values */ // AINP - AINN
#define ADS1219_MUX_DIFF_0_1 0x00 // AIN0 - AIN1
#define ADS1219_MUX_DIFF_2_3 0x20 // AIN2 - AIN3
#define ADS1219_MUX_DIFF_1_2 0x40 // AIN1 - AIN2
#define ADS1219_MUX_SINGLE_0 0x60 // AIN0 - AGND
#define ADS1219_MUX_SINGLE_1 0x80 // AIN1 - AGND
#define ADS1219_MUX_SINGLE_2 0xA0 // AIN2 - AGND
#define ADS1219_MUX_SINGLE_3 0xC0 // AIN3 - AGND
#define ADS1219_MUX_SHORTED 0xE0 // AVDD/2 - AVDD/2

/* Gain settings */
#define ADS1219_GAIN_ONE 1
#define ADS1219_GAIN_FOUR 4

/* Voltage reference */
#define ADS1219_VREF_INTERNAL 0
#define ADS1219_VREF_EXTERNAL 1

/* Data-rate settings */
#define ADS1219_DATARATE_20SPS 0
#define ADS1219_DATARATE_90SPS 1
#define ADS1219_DATARATE_330SPS 2
#define ADS1219_DATARATE_1000SPS 3

/* Conversion mode */
#define ADS1219_CM_SINGLE_SHOT 0
#define ADS1219_CM_CONTINUOUS 1

#endif

#ifndef LIS3MDL_h
#define LIS3MDL_h

#include <stdbool.h>
#include <stdint.h>

/* https://github.com/analogdevicesinc/no-OS/tree/main */

/*******************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Register Map fo ADXL38X (Rev H DS)*/
#define ADXL38X_DEVID_AD 0x00
#define ADXL38X_DEVID_MST 0x01
#define ADXL38X_PART_ID 0x02
#define ADXL38X_PART_ID_REV_ID 0x03
#define ADXL38X_SERIAL_NUMBER_0 0x04
#define ADXL38X_SERIAL_NUMBER_1 0x05
#define ADXL38X_SERIAL_NUMBER_2 0x06
#define ADXL38X_SERIAL_NUMBER_3 0x07
#define ADXL38X_SERIAL_NUMBER_4 0x08
#define ADXL38X_SERIAL_NUMBER_5 0x09
#define ADXL38X_SERIAL_NUMBER_6 0x0A
#define ADXL38X_DEV_DELTA_Q_X 0x0B
#define ADXL38X_DEV_DELTA_Q_Y 0x0C
#define ADXL38X_DEV_DELTA_Q_Z 0x0D
#define ADXL38X_DEV_DELTA_F0_X 0x0E
#define ADXL38X_DEV_DELTA_F0_Y 0x0F
#define ADXL38X_DEV_DELTA_F0_Z 0x10
#define ADXL38X_STATUS0 0x11
#define ADXL38X_STATUS1 0x12
#define ADXL38X_STATUS2 0x13
#define ADXL38X_STATUS3 0x14
#define ADXL38X_XDATA_H 0x15
#define ADXL38X_XDATA_L 0x16
#define ADXL38X_YDATA_H 0x17
#define ADXL38X_YDATA_L 0x18
#define ADXL38X_ZDATA_H 0x19
#define ADXL38X_ZDATA_L 0x1A
#define ADXL38X_TDATA_H 0x1B
#define ADXL38X_TDATA_L 0x1C
#define ADXL38X_FIFO_DATA 0x1D
#define ADXL38X_FIFO_STATUS0 0x1E
#define ADXL38X_FIFO_STATUS1 0x1F
#define ADXL38X_MISC0 0x20
#define ADXL38X_MISC1 0x21
#define ADXL38X_SENS_DSM 0x24
#define ADXL38X_CLK_CTRL 0x25
#define ADXL38X_OP_MODE 0x26
#define ADXL38X_DIG_EN 0x27
#define ADXL38X_SAR_I2C 0x28
#define ADXL38X_NVM_CTL 0x29
#define ADXL38X_REG_RESET 0x2A
#define ADXL38X_INT0_MAP0 0x2B
#define ADXL38X_INT0_MAP1 0x2C
#define ADXL38X_INT1_MAP0 0x2D
#define ADXL38X_INT1_MAP1 0x2E
#define ADXL38X_TEST_EN 0x2F
#define ADXL38X_FIFO_CFG0 0x30
#define ADXL38X_FIFO_CFG1 0x31
#define ADXL38X_SPT_CFG0 0x32
#define ADXL38X_SPT_CFG1 0x33
#define ADXL38X_SPT_CFG2 0x34
#define ADXL38X_SYNC_CFG 0x35
#define ADXL38X_PDM_CFG 0x36
#define ADXL38X_ACT_INACT_CTL 0x37
#define ADXL38X_SNSR_AXIS_EN 0x38
#define ADXL38X_THRESH_ACT_H 0x39
#define ADXL38X_THRESH_ACT_L 0x3A
#define ADXL38X_TIME_ACT_H 0x3B
#define ADXL38X_TIME_ACT_M 0x3C
#define ADXL38X_TIME_ACT_L 0x3D
#define ADXL38X_THRESH_INACT_H 0x3E
#define ADXL38X_THRESH_INACT_L 0x3F
#define ADXL38X_TIME_INACT_H 0x40
#define ADXL38X_TIME_INACT_M 0x41
#define ADXL38X_TIME_INACT_L 0x42
#define ADXL38X_TAP_THRESH 0x43
#define ADXL38X_TAP_DUR 0x44
#define ADXL38X_TAP_LATENT 0x45
#define ADXL38X_TAP_WINDOW 0x46
#define ADXL38X_TAP_CFG 0x47
#define ADXL38X_OR_CFG 0x48
#define ADXL38X_TRIG_CFG 0x49
#define ADXL38X_X_SAR_OFFSET 0x4A
#define ADXL38X_Y_SAR_OFFSET 0x4B
#define ADXL38X_Z_SAR_OFFSET 0x4C
#define ADXL38X_X_DSM_OFFSET 0x4D
#define ADXL38X_Y_DSM_OFFSET 0x4E
#define ADXL38X_Z_DSM_OFFSET 0x4F
#define ADXL38X_FILTER 0x50
#define ADXL38X_USER_TEMP_SENS_0 0x55
#define ADXL38X_USER_TEMP_SENS_1 0x56
#define ADXL38X_MISO 0x58
#define ADXL38X_SOUT0 0x59
#define ADXL38X_MCLK 0x5A
#define ADXL38X_BCLK 0x5B
#define ADXL38X_FSYNC 0x5C
#define ADXL38X_INT0 0x5D
#define ADXL38X_INT1 0x5E

/* Register reset value for ADXL38X */
#define ADXL38X_RESET_ZERO 0x00
#define ADXL38X_RESET_DEVID_AD 0xAD
#define ADXL38X_RESET_DEVID_MST 0x1D
#define ADXL38X_RESET_PART_ID 0x17
#define ADXL38X_RESET_PART_ID_REV_ID 0xC1
#define ADXL38X_RESET_STATUS0 0x80
#define ADXL38X_RESET_STATUS2 0x04
#define ADXL38X_RESET_INT0_MAP0 0x80
#define ADXL38X_RESET_INT1_MAP1 0x80
#define ADXL38X_RESET_SPT_CFG1 0x08
#define ADXL38X_RESET_SPT_CFG2 0x1A

/* Register masks */
#define ADXL38X_MASK_RANGE 0xC0
#define ADXL38X_MASK_OP_MODE 0x0F
#define ADXL38X_MASK_CHEN_DIG_EN 0xF0
#define ADXL38X_SLF_TST_CTRL_MSK 0xE0
#define ADXL38X_FIFOCFG_FIFOMODE_MSK 0x30

/* Pre-defined codes */
#define ADXL38X_RESET_CODE 0x52
#define ADXL38X_RESET_STATUS 0x80000400

/* Scale Factors */
/*
 * ADXL380
 * At +/- 4g with 16-bit resolution, scale from datasheet = 133.3ug/LSB.
 * 1333 * (10^-7)
 * For +/- 8g range a multiplier with value 2 is used.
 * For +/-16g range, a multiplier with value 4 is used.
 */
#define ADXL380_ACC_SCALE_FACTOR_GEE_MUL (int64_t)1333
/*
 * ADXL382
 * At +/- 15g with 16-bit resolution, scale from datasheet = 500ug/LSB.
 * 5000 * (10^-7)
 * For +/- 30g range a multiplier with value 2 is used.
 * For +/-60g range, a multiplier with value 4 is used.
 */
#define ADXL382_ACC_SCALE_FACTOR_GEE_MUL (int64_t)5000
/* Common denominator for ADXL380 and ADXL382 */
#define ADXL38X_ACC_SCALE_FACTOR_GEE_DIV (int32_t)10000000
/*
 * ADXL380/382 Temperature
 *
 * scale is 10.2 LSB/C.
 * Offset = 550LSB at 25C
 * offset at 0C = 550 - (10.2 * 25) LSB
 * offset at 0C = 295 LSB
 */
#define ADXL38X_TEMP_OFFSET (int32_t)295
#define ADXL38X_TEMP_SCALE_NUM (int32_t)102
#define ADXL38X_TEMP_SCALE_DEN (int32_t)10

/* Sensitivity */
/*
 * ADXL380
 * At +/- 4g with 16-bit resolution, sensitivity from datasheet = 7500LSB/g.
 * For +/- 8g range a divisor with value 2 is used.
 * For +/-16g range, a divisor with value 4 is used.
 */
#define ADXL380_ACC_SENSITIVITY (int32_t)7500
/*
 * ADXL382
 * At +/- 15g with 16-bit resolution, sensitivity from datasheet = 2000LSB/g.
 * For +/- 30g range a divisor with value 2 is used.
 * For +/-60g range, a divisor with value 4 is used.
 */
#define ADXL382_ACC_SENSITIVITY (int32_t)2000

/* Self Test Limits */
/* ADXL380 */
#define ADXL380_XY_ST_LIMIT_MIN 29
#define ADXL380_Z_ST_LIMIT_MIN 27
#define ADXL380_XY_ST_LIMIT_MAX 50
#define ADXL380_Z_ST_LIMIT_MAX 32
#define ADXL38X_ST_LIMIT_DENOMINATOR 10

/**
 * @struct adxl38x_range
 * @brief ADXL38X Range (OP MODE[7:6]).
 */
typedef enum {
	ADXL380_RANGE_4G = 0,
	ADXL380_RANGE_8G = 1,
	ADXL380_RANGE_16G = 2
} adxl38x_range_t;

/**
 * @struct adxl38x_op_mode
 * @brief ADXL38X Operating modes (OP MODE[3:0]).
 */
typedef enum {
	ADXL38X_MODE_STDBY = 0,
	ADXL38X_MODE_HRT_SND = 1,
	ADXL38X_MODE_ULP = 2,
	ADXL38X_MODE_VLP = 3,
	ADXL38X_MODE_LP = 4,
	ADXL38X_MODE_LP_SERIAL_ULP_OP = 6,
	ADXL38X_MODE_LP_SERIAL_VLP_OP = 7,
	ADXL38X_MODE_RBW = 8,
	ADXL38X_MODE_RBW_SERIAL_ULP_OP = 10,
	ADXL38X_MODE_RBW_SERIAL_VLP_OP = 11,
	ADXL38X_MODE_HP = 12,
	ADXL38X_MODE_HP_SERIAL_ULP_OP = 14,
	ADXL38X_MODE_HP_SERIAL_VLP_OP = 15
} adxl38x_op_mode_t;

/**
 * @struct adxl38x_channel_slct
 * @brief ADXL38X Channel enable (DIG_EN[7:4] - TZYX).
 */
typedef enum {
	ADXL38X_CH_DSB_ALL = 0,
	ADXL38X_CH_EN_X = 1,
	ADXL38X_CH_EN_Y = 2,
	ADXL38X_CH_EN_XY = 3,
	ADXL38X_CH_EN_Z = 4,
	ADXL38X_CH_EN_YZ = 6,
	ADXL38X_CH_EN_XYZ = 7,
	ADXL38X_CH_EN_T = 8,
	ADXL38X_CH_EN_ZT = 12,
	ADXL38X_CH_EN_YZT = 14,
	ADXL38X_CH_EN_XYZT = 15
} adxl38x_ch_select_t;

/**
 * @struct adxl38x_fifo_mode
 * @brief ADXL38X FIFO mode (FIFO_CFG1[5:4]).
 */
typedef enum {
	ADXL38X_FIFO_DISABLE = 0,
	ADXL38X_FIFO_NORMAL = 1,
	ADXL38X_FIFO_STREAM = 2,
	ADXL38X_FIFO_TRIGGER = 3
} adxl38x_fifo_mode_t;

typedef struct {
	// Status 3
	uint8_t DATA_READY : 1;
	uint8_t RESERVED2 : 7;
	// Status 2
	uint8_t UV_FLAG : 1;
	uint8_t UV_FLAG_STICKY : 1;
	uint8_t RESERVED1 : 1;
	uint8_t NVM_CRC_ERR : 1;
	uint8_t NVM_CRC_DONE : 1;
	uint8_t NVM_ECC_DET : 1;
	uint8_t RESERVED : 1;
	uint8_t NVM_ECC_DONE : 1;
	// Status 1
	uint8_t SINGLE_TAP : 1;
	uint8_t DOUBLE_TAP : 1;
	uint8_t TRIPLE_TAP : 1;
	uint8_t OVER_RANGE : 1;
	uint8_t OVER_RANGE_STICKY : 1;
	uint8_t ACT : 1;
	uint8_t INACT : 1;
	uint8_t NVM_IRQ : 1;
	// Status 0
	uint8_t PARITY_ERR_STICKY : 1;
	uint8_t FIFO_FULL : 1;
	uint8_t FIFO_OVR : 1;
	uint8_t FIFO_WATERMARK : 1;
	uint8_t FIFO_READY : 1;
	uint8_t EFUSE_BUSY_REGERR_STICKY : 1;
	uint8_t NVM_DONE : 1;
	uint8_t NVM_BUSY_STATUS : 1;
} _adxl38x_sts_reg_flags_t;
#endif

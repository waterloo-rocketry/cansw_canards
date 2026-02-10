#ifndef LPS_h
#define LPS_h

// register addresses
// Note: where register names differ between the register mapping table and
// the register descriptions in the datasheets, the names from the register
// descriptions are used here.

// Note: register names are prefixed with LPS to avoid name conflicts
typedef enum {
	LPS_REF_P_XL = 0x08,

	LPS_IF_CTRL = 0x0E, // 22DF

	LPS_WHO_AM_I = 0x0F,

	LPS_RES_CONF = 0x10,

	LPS_FIFO_WTM = 0x15, // 22DF

	LPS_I3C_IF_CTRL_ADD = 0x19, // 22DF

	LPS_FIFO_STATUS1 = 0x25, // 22DF
	LPS_FIFO_STATUS2 = 0x26, // 22DF
	LPS_STATUS_REG = 0x27,

	LPS_PRESS_OUT_XL = 0x28,
	LPS_PRESS_OUT_L = 0x29,
	LPS_PRESS_OUT_H = 0x2A,

	LPS_TEMP_OUT_L = 0x2B,
	LPS_TEMP_OUT_H = 0x2C,

	LPS_FIFO_STATUS = 0x2F, // 25H

	LPS_AMP_CTRL = 0x30, // 331AP

	LPS_DELTA_PRESS_XL = 0x3C, // 331AP
	LPS_DELTA_PRESS_L = 0x3D, // 331AP
	LPS_DELTA_PRESS_H = 0x3E, // 331AP

	LPS_FIFO_DATA_OUT_PRESS_XL = 0x78, // 22DF
	LPS_FIFO_DATA_OUT_PRESS_L = 0x79, // 22DF
	LPS_FIFO_DATA_OUT_PRESS_H = 0x7A, // 22DF

	// dummy addresses for registers in different locations on different devices;
	// the library translates these based on device type
	// value with sign flipped is used as index into translated_regs array

	LPS_REF_P_L = -1,
	LPS_REF_P_H = -2,
	LPS_CTRL_REG1 = -3,
	LPS_CTRL_REG2 = -4,
	LPS_CTRL_REG3 = -5,
	LPS_CTRL_REG4 = -6,
	LPS_INTERRUPT_CFG = -7,
	LPS_INT_SOURCE = -8,
	LPS_FIFO_CTRL = -9,
	LPS_THS_P_L = -10,
	LPS_THS_P_H = -11,
	LPS_RPDS_L = -12,
	LPS_RPDS_H = -13,
	// update dummy_reg_count if registers are added here!

	// device-specific register addresses

	LPS331AP_REF_P_L = 0x09,
	LPS331AP_REF_P_H = 0x0A,
	LPS331AP_CTRL_REG1 = 0x20,
	LPS331AP_CTRL_REG2 = 0x21,
	LPS331AP_CTRL_REG3 = 0x22,
	LPS331AP_INTERRUPT_CFG = 0x23,
	LPS331AP_INT_SOURCE = 0x24,
	LPS331AP_THS_P_L = 0x25,
	LPS331AP_THS_P_H = 0x26,

	LPS25H_REF_P_L = 0x09,
	LPS25H_REF_P_H = 0x0A,
	LPS25H_CTRL_REG1 = 0x20,
	LPS25H_CTRL_REG2 = 0x21,
	LPS25H_CTRL_REG3 = 0x22,
	LPS25H_CTRL_REG4 = 0x23,
	LPS25H_INTERRUPT_CFG = 0x24,
	LPS25H_INT_SOURCE = 0x25,
	LPS25H_FIFO_CTRL = 0x2E,
	LPS25H_THS_P_L = 0x30,
	LPS25H_THS_P_H = 0x31,
	LPS25H_RPDS_L = 0x39,
	LPS25H_RPDS_H = 0x3A,

	LPS22DF_INTERRUPT_CFG = 0x0B,
	LPS22DF_THS_P_L = 0x0C,
	LPS22DF_THS_P_H = 0x0D,
	LPS22DF_CTRL_REG1 = 0x10,
	LPS22DF_CTRL_REG2 = 0x11,
	LPS22DF_CTRL_REG3 = 0x12,
	LPS22DF_CTRL_REG4 = 0x13,
	LPS22DF_FIFO_CTRL = 0x14,
	LPS22DF_REF_P_L = 0x16,
	LPS22DF_REF_P_H = 0x17,
	LPS22DF_RPDS_L = 0x1A,
	LPS22DF_RPDS_H = 0x1B,
	LPS22DF_INT_SOURCE = 0x24
} LPS_reg;

#endif
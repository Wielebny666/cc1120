/*
 * cc112x_enums.h
 *
 *  Created on: 9 cze 2020
 *      Author: kurzawa.p
 */

#ifndef CC112X_ENUMS_H
#define CC112X_ENUMS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef enum
{
	STATE_IDLE = 0,
	STATE_RX = 1,
	STATE_TX = 2,
	STATE_FSTXON = 3,
	STATE_CALIBRATE,
	STATE_SETTLING,
	STATE_RX_FIFO_ERROR,
	STATE_TX_FIFO_ERROR,
} state_t;

typedef enum
{
	MARCSTATE_SLEEP = 0b00000,
	MARCSTATE_IDDLE = 0b00001,
	MARCSTATE_XOFF = 0b00010,
	MARCSTATE_BIAS_SETTLE_MC = 0b00011,
	MARCSTATE_REG_SETTLE_MC = 0b00100,
	MARCSTATE_MANCAL = 0b00101,
	MARCSTATE_BIAS_SETTLE = 0b00110,
	MARCSTATE_REG_SETTLE = 0b00111,
	MARCSTATE_STARTCAL = 0b01000,
	MARCSTATE_BWBOOST = 0b01001,
	MARCSTATE_FS_LOCK = 0b01010,
	MARCSTATE_IFADCON = 0b01011,
	MARCSTATE_ENDCAL = 0b01100,
	MARCSTATE_RX = 0b01101,
	MARCSTATE_RX_END = 0b01110,
	MARCSTATE_RESERVED = 0b01111,
	MARCSTATE_TXRX_SWITCH = 0b10000,
	MARCSTATE_RX_FIFO_ERR = 0b10001,
	MARCSTATE_FSTXON = 0b10010,
	MARCSTATE_TX = 0b10011,
	MARCSTATE_TX_END = 0b10100,
	MARCSTATE_RXTX_SWITCH = 0b10101,
	MARCSTATE_TX_FIFO_ERR = 0b10110,
	MARCSTATE_IFADCON_TXRX = 0b10111,
}marcstate_e;

typedef enum
{
	SERIAL_RX = 9,
	RSSI_VALID = 13,
	CARRIER_SENSE_VALID = 16,
	CARRIER_SENSE = 17,
	LNA_PA_REG_PD = 23,
	LNA_PD = 24,
	PA_PD = 25,
} gpiox_cfg_t;

typedef enum
{
	TRANSPARENT_X1,
	TRANSPARENT_X2,
	TRANSPARENT_X4,
} transparent_infact_t;

typedef enum
{
	MOD_2FSK,
	MOD_2GFSK,
	MOD_ASK_OOK = 3,
	MOD_4FSK,
	MOD_4GFSK
} mod_format_t;

typedef enum
{
	PN_CC1121 = 0x40,
	PN_CC1120 = 0x48,
	PN_CC1125 = 0x58,
	PN_CC1175 = 0x5A
} partnumber_t;

typedef enum
{
	DATA_RATE_1_2K,
	DATA_RATE_2_4K,
	DATA_RATE_4_8K,
	DATA_RATE_9_6K,
	DATA_RATE_19_2K,
	DATA_RATE_38_4K,
	DATA_RATE_50K,
	DATA_RATE_100K,
	DATA_RATE_150K,
	DATA_RATE_200K,
	DATA_RATE_MAX
} data_rate_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CC112X_ENUMS_H */

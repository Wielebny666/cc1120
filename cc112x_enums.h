/*
 * cc112x_enums.h
 *
 *  Created on: 9 cze 2020
 *      Author: kurzawa.p
 */

#ifndef CC112X_ENUMS_H
#define CC112X_ENUMS_H

#ifdef __cplusplus
extern "C" {
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
	IDLE = 0,
	RX = 1,
	TX = 2,
	FSTXON = 3,
	CALIBRATE,
	SETTLING,
	RX_FIFO_ERROR,
	TX_FIFO_ERROR,
} state_t;

typedef enum
{
	SERIAL_RX = 9,
	RSSI_VALID = 13,
	CARRIER_SENSE_VALID = 16,
	CARRIER_SENSE = 17,
	LNA_PA_REG_PD = 23,
	LNA_PD  = 24,
	PA_PD = 25,
} gpiox_cfg_t;

typedef enum
{
	TRANSPARENT_X1,
	TRANSPARENT_X2,
	TRANSPARENT_X4,
}transparent_infact_t;

typedef enum
{
	MOD_2FSK,
	MOD_2GFSK,
	MOD_ASK_OOK = 3,
	MOD_4FSK,
	MOD_4GFSK
}mod_format_t;

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

/*
 * cc112x_cfg.h
 *
 *  Created on: 30 maj 2020
 *      Author: kurza
 */

#ifndef CC112X_CFG_H
#define CC112X_CFG_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include "cc112x_defs.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  GLOBAL VARIABLES
 **********************/

// Address Config = No address check 
// Bit Rate = 4.8 
// Carrier Frequency = 870.000000 
// Deviation = 17.944336 
// Device Address = 0 
// Manchester Enable = false 
// Modulation Format = ASK/OOK 
// PA Ramping = true 
// Packet Bit Length = 0 
// Packet Length = 255 
// Packet Length Mode = Variable 
// Performance Mode = High Performance 
// RX Filter BW = 66.666667 
// Symbol rate = 4.8 
// TX Power = 27 
// Whitening = false 

static const cc112x_reg_setting_t test[]= 
{
	{CC112X_DEVIATION_M,       0x26},
	{CC112X_MODCFG_DEV_E,      0x1D},
	{CC112X_DCFILT_CFG,        0x13},
	{CC112X_PREAMBLE_CFG1,     0x18},
	{CC112X_PREAMBLE_CFG0,     0x33},
	{CC112X_IQIC,              0x00},
	{CC112X_CHAN_BW,           0x03},
	{CC112X_MDMCFG0,           0x04},
	{CC112X_SYMBOL_RATE2,      0x63},
	{CC112X_AGC_REF,           0x30},
	{CC112X_AGC_CS_THR,        0xEC},
	{CC112X_AGC_CFG3,          0xD1},
	{CC112X_AGC_CFG2,          0x3F},
	{CC112X_AGC_CFG1,          0x32},
	{CC112X_AGC_CFG0,          0x9F},
	{CC112X_FIFO_CFG,          0x00},
	{CC112X_FS_CFG,            0x12},
	// {CC112X_PKT_CFG0,          0x20},
	//{CC112X_PA_CFG2,           0x7C}, //PA RAMPING - ON
	{CC112X_PA_CFG2,           0x3C}, //PA RAMPING - OFF
	{CC112X_PA_CFG0,           0x7E},
	{CC112X_PKT_LEN,           0xFF},
	{CC112X_IF_MIX_CFG,        0x00},
	{CC112X_FREQOFF_CFG,       0x00},
	{CC112X_TOC_CFG,           0x0A},
	{CC112X_FREQ2,             0x6C},
	{CC112X_FREQ1,             0xC0},
	{CC112X_FS_DIG1,           0x00},
	{CC112X_FS_DIG0,           0x5F},
	{CC112X_FS_CAL1,           0x40},
	{CC112X_FS_CAL0,           0x0E},
	{CC112X_FS_DIVTWO,         0x03},
	{CC112X_FS_DSM0,           0x33},
	{CC112X_FS_DVC0,           0x17},
	{CC112X_FS_PFD,            0x50},
	{CC112X_FS_PRE,            0x6E},
	{CC112X_FS_REG_DIV_CML,    0x14},
	{CC112X_FS_SPARE,          0xAC},
	{CC112X_FS_VCO0,           0xB4},
	{CC112X_XOSC5,             0x0E},
	{CC112X_XOSC1,             0x03},
};


	// Address Config = No address check
	// Bit Rate = 10
	// Carrier Frequency = 868.000000
	// Deviation = 100.097656
	// Device Address = 0
	// Manchester Enable = false
	// Modulation Format = 2-FSK
	// PA Ramping = true
	// Packet Bit Length = 0
	// Packet Length = 3
	// Packet Length Mode = Variable
	// Performance Mode = High Performance
	// RX Filter BW = 20.000000
	// Symbol rate = 10
	// TX Power = 27
	// Whitening = false

	// @formatter:off
	static cc112x_reg_setting_t cc112x_tx_reg_setting[] =
		{
			{CC112X_IOCFG3, 0x59}, //PA_EN
			{CC112X_IOCFG2, 0x58}, //LNA_EN
			{CC112X_IOCFG0, 0x09}, //TX_INPUT
			{CC112X_SYNC_CFG1, 0x0B},

			{CC112X_DEVIATION_M, 0x48}, //20kHz
			{CC112X_MODCFG_DEV_E, 0x05},

			{CC112X_DCFILT_CFG, 0x1C},
			{CC112X_PREAMBLE_CFG1, 0x00},

			{CC112X_IQIC, 0xC6},
			{CC112X_CHAN_BW, 0xD5},

			{CC112X_MDMCFG1, 0x06},
			{CC112X_MDMCFG0, 0x65},

			//		{CC112X_SYMBOL_RATE2,      0x74},
			//		{CC112X_SYMBOL_RATE1,      0x7A},
			//		{CC112X_SYMBOL_RATE0,      0xE1},

			{CC112X_AGC_REF, 0x20},
			// Carrier sense threshold -90 dBm = 0x0C
			// Carrier sense threshold -123 dBm = 0xED
			// Carrier sense threshold -80 dBm = 0x16
			// Carrier sense threshold -70 dBm = 0x20
			{CC112X_AGC_CS_THR, 0x34},
			//{CC112X_AGC_GAIN_ADJUST,   0xBA},

			{CC112X_AGC_CFG1, 0xA9},
			{CC112X_AGC_CFG0, 0xCF},
			{CC112X_FIFO_CFG, 0x00},
			{CC112X_FS_CFG, 0x12},
			{CC112X_PKT_CFG2, 0x07},
			{CC112X_PKT_CFG1, 0x00},
			{CC112X_PKT_CFG0, 0x20},
			{CC112X_PA_CFG2, 0x77},
			{CC112X_IF_MIX_CFG, 0x00},
			{CC112X_FREQOFF_CFG, 0x22},

			{CC112X_FREQ2, 0x6C}, //870MHz
			{CC112X_FREQ1, 0xC0},
			{CC112X_FREQ0, 0x00},

			{CC112X_FS_DIG1, 0x00},
			{CC112X_FS_DIG0, 0x5F},
			{CC112X_FS_CAL1, 0x40},
			{CC112X_FS_CAL0, 0x0E},
			{CC112X_FS_DIVTWO, 0x03},
			{CC112X_FS_DSM0, 0x33},
			{CC112X_FS_DVC0, 0x17},
			{CC112X_FS_PFD, 0x50},
			{CC112X_FS_PRE, 0x6E},
			{CC112X_FS_REG_DIV_CML, 0x14},
			{CC112X_FS_SPARE, 0xAC},
			{CC112X_FS_VCO0, 0xB4},
			{CC112X_XOSC5, 0x0E},
			{CC112X_XOSC1, 0x03},
			{CC112X_SERIAL_STATUS, 0x08},
	};

	static cc112x_reg_setting_t cc112x_rx_reg_setting[] =
		{
			{CC112X_IOCFG3, 0x59}, //PA_EN
			{CC112X_IOCFG2, 0x58}, //LNA_EN
			{CC112X_IOCFG0, 0x09}, //TX_INPUT
			{CC112X_SYNC_CFG1, 0x0B},

			{CC112X_DEVIATION_M, 0x9A},
			{CC112X_MODCFG_DEV_E, 0x07},

			{CC112X_DCFILT_CFG, 0x1C},
			{CC112X_PREAMBLE_CFG1, 0x00},

			{CC112X_IQIC, 0xC6},
			{CC112X_CHAN_BW, 0x0A},

			{CC112X_MDMCFG1, 0x06},
			{CC112X_MDMCFG0, 0x65},

			{CC112X_SYMBOL_RATE2, 0x74},
			{CC112X_SYMBOL_RATE1, 0x7A},
			{CC112X_SYMBOL_RATE0, 0xE1},

			{CC112X_AGC_REF, 0x20},
			{CC112X_AGC_CS_THR, 0x01},
			{CC112X_AGC_CFG2, 0x07},
			{CC112X_AGC_CFG1, 0x00},
			{CC112X_AGC_CFG0, 0xCF},
			{CC112X_FIFO_CFG, 0x00},
			{CC112X_FS_CFG, 0x12},
			{CC112X_PKT_CFG2, 0x07},
			{CC112X_PKT_CFG1, 0x00},
			{CC112X_PKT_CFG0, 0x20},
			{CC112X_PA_CFG2, 0x77},
			{CC112X_IF_MIX_CFG, 0x00},
			{CC112X_FREQOFF_CFG, 0x22},
			{CC112X_FREQ2, 0x6C},
			{CC112X_FREQ1, 0x80},
			{CC112X_FS_DIG1, 0x00},
			{CC112X_FS_DIG0, 0x5F},
			{CC112X_FS_CAL1, 0x40},
			{CC112X_FS_CAL0, 0x0E},
			{CC112X_FS_DIVTWO, 0x03},
			{CC112X_FS_DSM0, 0x33},
			{CC112X_FS_DVC0, 0x17},
			{CC112X_FS_PFD, 0x50},
			{CC112X_FS_PRE, 0x6E},
			{CC112X_FS_REG_DIV_CML, 0x14},
			{CC112X_FS_SPARE, 0xAC},
			{CC112X_FS_VCO0, 0xB4},
			{CC112X_XOSC5, 0x0E},
			{CC112X_XOSC1, 0x03},
			{CC112X_SERIAL_STATUS, 0x08},
	};

	static cc112x_reg_setting_t cc112x_test_reg_setting[] =
		{
			{CC112X_IOCFG3, 0x59}, //PA_EN
			{CC112X_IOCFG2, 0x58}, //LNA_EN
			{CC112X_IOCFG0, 0x09}, //TX_INPUT
			{CC112X_SYNC_CFG1, 0x0B},

			{CC112X_DEVIATION_M, 0x9A},
			{CC112X_MODCFG_DEV_E, 0x07},

			{CC112X_DCFILT_CFG, 0x1C},
			{CC112X_PREAMBLE_CFG1, 0x18}, //?
			{CC112X_IQIC, 0xC6},
			{CC112X_CHAN_BW, 0x08},
			{CC112X_MDMCFG1, 0x06}, //?
			{CC112X_MDMCFG0, 0x6A}, //?
			{CC112X_AGC_REF, 0x20},
			{CC112X_AGC_CS_THR, 0x19},
			{CC112X_AGC_CFG1, 0x00}, //?
			{CC112X_AGC_CFG0, 0xCF},
			{CC112X_FIFO_CFG, 0x00},
			{CC112X_SETTLING_CFG, 0x03},
			{CC112X_FS_CFG, 0x12},
			{CC112X_PKT_CFG2, 0x07},
			{CC112X_PKT_CFG0, 0x20},
			{CC112X_PA_CFG2, 0x5D},
			{CC112X_PKT_LEN, 0xFF},
			{CC112X_IF_MIX_CFG, 0x00},
			{CC112X_FREQOFF_CFG, 0x22},
			{CC112X_FREQ2, 0x6C},
			{CC112X_FREQ1, 0x80},
			{CC112X_FS_DIG1, 0x00},
			{CC112X_FS_DIG0, 0x5F},
			{CC112X_FS_CAL1, 0x40},
			{CC112X_FS_CAL0, 0x0E},
			{CC112X_FS_DIVTWO, 0x03},
			{CC112X_FS_DSM0, 0x33},
			{CC112X_FS_DVC0, 0x17},
			{CC112X_FS_PFD, 0x50},
			{CC112X_FS_PRE, 0x6E},
			{CC112X_FS_REG_DIV_CML, 0x14},
			{CC112X_FS_SPARE, 0xAC},
			{CC112X_FS_VCO0, 0xB4},
			{CC112X_XOSC5, 0x0E},
			{CC112X_XOSC1, 0x03},
	};

	// Address Config = No address check
	// Bit Rate = 10
	// Carrier Frequency = 870.000000
	// Deviation = 17.944336
	// Device Address = 0
	// Manchester Enable = false
	// Modulation Format = ASK/OOK
	// PA Ramping = true
	// Packet Bit Length = 0
	// Packet Length = 3
	// Packet Length Mode = Variable
	// Performance Mode = High Performance
	// RX Filter BW = 66.666667
	// Symbol rate = 10
	// TX Power = 27
	// Whitening = false

	static cc112x_reg_setting_t preferredSettings[] =
		{
			{CC112X_SYNC3, 0xAA},
			{CC112X_SYNC2, 0xAA},
			{CC112X_SYNC1, 0xAA},
			{CC112X_SYNC0, 0xAA},
			{CC112X_SYNC_CFG1, 0x1F},
			{CC112X_DEVIATION_M, 0x26},
			{CC112X_MODCFG_DEV_E, 0x1D},
			{CC112X_DCFILT_CFG, 0x13},
			{CC112X_PREAMBLE_CFG1, 0x00},
			{CC112X_PREAMBLE_CFG0, 0x33},
			{CC112X_IQIC, 0x00},
			{CC112X_CHAN_BW, 0x03},
			{CC112X_MDMCFG1, 0x06},
			{CC112X_MDMCFG0, 0x4A},
			{CC112X_SYMBOL_RATE2, 0x74},
			{CC112X_SYMBOL_RATE1, 0x7A},
			{CC112X_SYMBOL_RATE0, 0xE1},
			{CC112X_AGC_REF, 0x30},
			{CC112X_AGC_CS_THR, 0xEC},
			{CC112X_AGC_CFG3, 0xD1},
			{CC112X_AGC_CFG2, 0x3F},
			{CC112X_AGC_CFG1, 0x0A},
			{CC112X_AGC_CFG0, 0x9F},
			{CC112X_FIFO_CFG, 0x00},
			{CC112X_FS_CFG, 0x12},
			{CC112X_PKT_CFG2, 0x07},
			{CC112X_PKT_CFG1, 0x00},
			{CC112X_PKT_CFG0, 0x20},
			{CC112X_PA_CFG2, 0x7F},
			{CC112X_PA_CFG0, 0x7D},
			{CC112X_IF_MIX_CFG, 0x00},
			{CC112X_FREQOFF_CFG, 0x22},
			{CC112X_TOC_CFG, 0x0A},
			{CC112X_CFM_DATA_CFG, 0x01},
			{CC112X_FREQ2, 0x6C},
			{CC112X_FREQ1, 0xC0},
			{CC112X_FS_DIG1, 0x00},
			{CC112X_FS_DIG0, 0x5F},
			{CC112X_FS_CAL1, 0x40},
			{CC112X_FS_CAL0, 0x0E},
			{CC112X_FS_DIVTWO, 0x03},
			{CC112X_FS_DSM0, 0x33},
			{CC112X_FS_DVC0, 0x17},
			{CC112X_FS_PFD, 0x50},
			{CC112X_FS_PRE, 0x6E},
			{CC112X_FS_REG_DIV_CML, 0x14},
			{CC112X_FS_SPARE, 0xAC},
			{CC112X_FS_VCO0, 0xB4},
			{CC112X_XOSC5, 0x0E},
			{CC112X_XOSC1, 0x03},
			{CC112X_SERIAL_STATUS, 0x08},
	};

	// Address Config = No address check
	// Bit Rate = 100
	// Carrier Frequency = 868.000000
	// Deviation = 17.944336
	// Device Address = 0
	// Manchester Enable = false
	// Modulation Format = ASK/OOK
	// PA Ramping = true
	// Packet Bit Length = 0
	// Packet Length = 3
	// Packet Length Mode = Variable
	// Performance Mode = High Performance
	// RX Filter BW = 200.000000
	// Symbol rate = 100
	// TX Power = 27
	// Whitening = false

	static cc112x_reg_setting_t ook_870_transparent_rx[] =
		{
			{CC112X_DEVIATION_M, 0x26},
			{CC112X_MODCFG_DEV_E, 0x1D},
			{CC112X_DCFILT_CFG, 0x1C},
			{CC112X_PREAMBLE_CFG1, 0x00},
			{CC112X_PREAMBLE_CFG0, 0x33},
			{CC112X_IQIC, 0x00},
			{CC112X_CHAN_BW, 0x03},
			{CC112X_MDMCFG1, 0x06},
			{CC112X_MDMCFG0, 0x65},
			{CC112X_SYMBOL_RATE2, 0x63},
			{CC112X_AGC_REF, 0x2B},
			{CC112X_AGC_CS_THR, 0x19},
			{CC112X_AGC_CFG3, 0xD1},
			{CC112X_AGC_CFG2, 0x3F},
			{CC112X_AGC_CFG1, 0x35}, //0x35}, //0x31},
			{CC112X_AGC_CFG0, 0x90}, //0x90}, //0x9F},
			{CC112X_FIFO_CFG, 0x00},
			{CC112X_FS_CFG, 0x12},
			{CC112X_PKT_CFG2, 0x07},
			{CC112X_PKT_CFG1, 0x00},
			{CC112X_PKT_CFG0, 0x20},
			//27dBm
			{CC112X_PA_CFG2,           0x7C}, //PA RAMPING - ON
			{CC112X_PA_CFG0,           0x7E},

			{CC112X_IF_MIX_CFG, 0x00},
			{CC112X_FREQOFF_CFG, 0x00},
			{CC112X_TOC_CFG, 0x0A},
			{CC112X_FREQ2, 0x6C},
			{CC112X_FREQ1, 0x80},
			{CC112X_FS_DIG1, 0x00},
			{CC112X_FS_DIG0, 0x5F},
			{CC112X_FS_CAL1, 0x40},
			{CC112X_FS_CAL0, 0x0E},
			{CC112X_FS_DIVTWO, 0x03},
			{CC112X_FS_DSM0, 0x33},
			{CC112X_FS_DVC0, 0x17},
			{CC112X_FS_PFD, 0x50},
			{CC112X_FS_PRE, 0x6E},
			{CC112X_FS_REG_DIV_CML, 0x14},
			{CC112X_FS_SPARE, 0xAC},
			{CC112X_FS_VCO0, 0xB4},
			{CC112X_XOSC5, 0x0E},
			{CC112X_XOSC1, 0x03},
			{CC112X_SERIAL_STATUS, 0x08},
	};
	// @formatter:on


// Address Config = No address check 
// Bit Rate = 10 
// Carrier Frequency = 870.000000 
// Deviation = 17.944336 
// Device Address = 0 
// Manchester Enable = false 
// Modulation Format = ASK/OOK 
// PA Ramping = true 
// Packet Bit Length = 0 
// Packet Length = 3 
// Packet Length Mode = Variable 
// Performance Mode = High Performance 
// RX Filter BW = 66.666667 
// Symbol rate = 10 
// TX Power = 27 
// Whitening = false 

static const  cc112x_reg_setting_t ook_870_transparent_rx_2[]= 
{
  {CC112X_SYNC3,             0xAA},
  {CC112X_SYNC2,             0xAA},
  {CC112X_SYNC1,             0xAA},
  {CC112X_SYNC0,             0xAA},
  {CC112X_SYNC_CFG1,         0x1F},
  {CC112X_DEVIATION_M,       0x26},
  {CC112X_MODCFG_DEV_E,      0x1D},
  {CC112X_DCFILT_CFG,        0x13},
  {CC112X_PREAMBLE_CFG1,     0x00},
  {CC112X_PREAMBLE_CFG0,     0x33},
  {CC112X_IQIC,              0x00},
  {CC112X_CHAN_BW,           0x03},
  {CC112X_MDMCFG1,           0x06},
  {CC112X_MDMCFG0,           0x4A},
  {CC112X_SYMBOL_RATE2,      0x74},
  {CC112X_SYMBOL_RATE1,      0x7A},
  {CC112X_SYMBOL_RATE0,      0xE1},
  {CC112X_AGC_REF,           0x30},
  {CC112X_AGC_CS_THR,        0xEC},
  {CC112X_AGC_CFG3,          0xD1},
  {CC112X_AGC_CFG2,          0x3F},
  {CC112X_AGC_CFG1,          0x0A},
  {CC112X_AGC_CFG0,          0x9F},
  {CC112X_FIFO_CFG,          0x00},
  {CC112X_FS_CFG,            0x12},
  {CC112X_PKT_CFG2,          0x07},
  {CC112X_PKT_CFG1,          0x00},
  {CC112X_PKT_CFG0,          0x20},
  {CC112X_PA_CFG2,           0x77},
  {CC112X_PA_CFG0,           0x6D},
  {CC112X_IF_MIX_CFG,        0x00},
  {CC112X_FREQOFF_CFG,       0x22},
  {CC112X_TOC_CFG,           0x0A},
  {CC112X_FREQ2,             0x6C},
  {CC112X_FREQ1,             0xC0},
  {CC112X_FS_DIG1,           0x00},
  {CC112X_FS_DIG0,           0x5F},
  {CC112X_FS_CAL1,           0x40},
  {CC112X_FS_CAL0,           0x0E},
  {CC112X_FS_DIVTWO,         0x03},
  {CC112X_FS_DSM0,           0x33},
  {CC112X_FS_DVC0,           0x17},
  {CC112X_FS_PFD,            0x50},
  {CC112X_FS_PRE,            0x6E},
  {CC112X_FS_REG_DIV_CML,    0x14},
  {CC112X_FS_SPARE,          0xAC},
  {CC112X_FS_VCO0,           0xB4},
  {CC112X_XOSC5,             0x0E},
  {CC112X_XOSC1,             0x03},
  {CC112X_SERIAL_STATUS,     0x08},
};

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CC112X_CFG_H */

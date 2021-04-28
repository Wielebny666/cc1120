/*
 * cc1120.c
 *
 *  Created on: 12 maj 2020
 *      Author: kurzawa.p
 */

/*********************
 *      INCLUDES
 *********************/
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_err.h"

#include "cc112x_cfg.h"
#include "cc112x_hal.h"
#include "cc112x.h"
#include "foreach.h"

/*********************
 *      DEFINES
 *********************/
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static int32_t cc112x_calculate_freq_error_est(uint16_t freq_reg_error);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "cc112x";

static uint8_t cc112x_1[0x2F];
static uint8_t cc112x_2[0x41];

/* Special configuration for different data rate.
 * [1.2k][2.4k][4.8k][9.6k][19.2k][38.4k][50k][100k][150k][200k][REG] 
 */
static const uint16_t cc112x_radio_cfg[][11] =
	{
		{0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x08, 0x07, 0x07, CC112X_SYNC_CFG1},
		{0x06, 0x06, 0x3B, 0x48, 0x48, 0x48, 0x99, 0x99, 0x53, 0x53, CC112X_DEVIATION_M},
		{0x03, 0x03, 0x04, 0x05, 0x05, 0x05, 0x05, 0x2D, 0x2F, 0x2F, CC112X_MODCFG_DEV_E},
		{0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x15, 0x15, 0x04, 0x04, CC112X_DCFILT_CFG},
		{0x18, 0x18, 0x18, 0x18, 0x18, 0x20, 0x18, 0x18, 0x18, 0x18, CC112X_PREAMBLE_CFG0},
		{0x18, 0x18, 0x18, 0x18, 0x18, 0x14, 0x18, 0x18, 0x18, 0x18, CC112X_PREAMBLE_CFG1},
		{0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x3A, 0x3A, 0x00, 0x00, CC112X_FREQ_IF_CFG},
		{0xC6, 0xC6, 0x46, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, CC112X_IQIC},
		{0x08, 0x08, 0x04, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, CC112X_CHAN_BW},
		{0x43, 0x53, 0x63, 0x73, 0x83, 0x93, 0x99, 0x99, 0xA3, 0xA9, CC112X_SYMBOL_RATE2},
		{0xA9, 0xA9, 0xA9, 0xA9, 0xA9, 0xA9, 0x99, 0x99, 0x33, 0x99, CC112X_SYMBOL_RATE1},
		{0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x99, 0x9A, 0x33, 0x99, CC112X_SYMBOL_RATE0},
		{0x20, 0x20, 0x20, 0x20, 0x20, 0x36, 0x3C, 0x3C, 0x3C, 0x3C, CC112X_AGC_REF},
		{0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0xEF, 0xEF, 0xEC, 0xEC, CC112X_AGC_CS_THR},
		{0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x91, 0x83, 0x83, CC112X_AGC_CFG3},
		{0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x60, 0x60, CC112X_AGC_CFG2},
		{0xCF, 0xCF, 0xCF, 0xCF, 0xCF, 0xCF, 0xC0, 0xC0, 0xC0, 0xC0, CC112X_AGC_CFG0},
		{0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, CC112X_FS_CFG},
		//		{0x7E, 0x7E, 0x7E, 0x7D, 0x7C, 0x7B, 0x79, 0x7B, 0x02, 0x01, CC112X_PA_CFG0},
		{0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x20, 0x20, CC112X_FREQOFF_CFG},
		{0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0A, 0x0A, 0x0A, 0x0A, CC112X_TOC_CFG},
};

/**********************
 *      MACROS
 **********************/
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void cc112x_init_config(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	//foreach (cc112x_reg_setting_t *cfg,  preferredSettings) //test) // //cc112x_tx_reg_setting) //cc112x_test_reg_setting)//
	foreach (cc112x_reg_setting_t *cfg, ook_870_transparent_tx)
	{
		cc112x_write_register(cfg->addr, cfg->data);
	}
}

void cc112x_choice_config(cc112x_radio_cfg_t choice)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	switch (choice)
	{
	case CFG_OOK_870_PACKET_TX:
	{
		foreach (cc112x_reg_setting_t *cfg, ook_870_packet_tx)
		{
			cc112x_write_register(cfg->addr, cfg->data);
		}
		break;
	}
	case CFG_OOK_870_ASYNC_TX:
	{
		foreach (cc112x_reg_setting_t *cfg, ook_870_transparent_tx)
		{
			cc112x_write_register(cfg->addr, cfg->data);
		}
		break;
	}
	case CFG_OOK_870_ASYNC_RX:
	{
		foreach (cc112x_reg_setting_t *cfg, ook_870_transparent_rx_2)
		{
			cc112x_write_register(cfg->addr, cfg->data);
		}
		break;
	}
	case CFG_OOK_880_ASYNC_TX:
	{
		foreach (cc112x_reg_setting_t *cfg, ook_880_transparent_tx)
		{
			cc112x_write_register(cfg->addr, cfg->data);
		}
		break;
	}
	case CFG_OOK_868_ASYNC_TX:
	{
		foreach (cc112x_reg_setting_t *cfg, ook_868_transparent_tx)
		{
			cc112x_write_register(cfg->addr, cfg->data);
		}
		break;
	}
	default:
		break;
	}
}

void cc112x_send_data(const uint8_t *buff, uint8_t len)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	marcstate_t marcstate;

	ESP_ERROR_CHECK(cc112x_write_register(CC112X_PKT_LEN, len));
	ESP_ERROR_CHECK(cc112x_write_burst_registers(CC112X_FIFO, buff, len));
	cc112x_set_tx_state();
	do
	{
		vTaskDelay(50 / portTICK_PERIOD_MS);
		cc112x_read_register(CC112X_MARCSTATE, &marcstate);
		ESP_LOGD(TAG, "MARC STATE %d", marcstate.marc_state);
	} while (marcstate.marc_state != MARCSTATE_IDDLE);

	cc112x_set_flush_tx();
}

void cc112x_set_radio_config(data_rate_t cfg)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	for (uint16_t i = 0; i < sizeof(cc112x_radio_cfg) / sizeof(cc112x_radio_cfg[0]); i++)
	{
		cc112x_write_register(cc112x_radio_cfg[i][10], cc112x_radio_cfg[i][cfg]);
	}
}

void cc112x_read_config(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	ESP_ERROR_CHECK(cc112x_read_burst_registers(0x0000, cc112x_1, 0x2F));
}

void cc112x_print_config(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	for (uint8_t i = 0; i < 0x2F; i++)
	{
		ESP_LOGI(TAG, "%#06X %#04X", i, cc112x_1[i]); //, BYTE_TO_BINARY(as3933.r0.reg);
	}
}

void cc112x_set_idle_state(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc112x_command_strobe(CC112X_SIDLE);
}

void cc112x_reset(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc112x_command_strobe(CC112X_SRES);
}

void cc112x_set_tx_state(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc112x_command_strobe(CC112X_STX);
}

void cc112x_set_flush_tx(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc112x_command_strobe(CC112X_SFTX);
}

void cc112x_set_rx_state(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc112x_command_strobe(CC112X_SRX);
}

void cc112x_set_flush_rx(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc112x_command_strobe(CC112X_SFRX);
}

bool cc112x_get_rssi_valid(void)
{
	rssi0_t rssi0;
	cc112x_read_register(CC112X_RSSI0, &rssi0.reg);
	return rssi0.rssi_valid;
}

state_t cc112x_get_chip_status(void)
{
	chip_status_t chip_status;
	cc112x_read_status(&chip_status.reg);
	return chip_status.state;
}

/**
 *
 * @fn         cc112x_get_8bit_rssi
 * @param       none
 *
 * @return      none
 */
int8_t cc112x_get_8bit_rssi(void)
{
	uint8_t rssi = 0;
	if (cc112x_get_rssi_valid())
	{
		cc112x_read_register(CC112X_RSSI1, &rssi);
		return (int8_t)rssi + RF_RSSI_OFFSET;
	}
	return 0;
}

float cc112x_get_12bit_rssi(void)
{
	uint8_t rssi2complMSB;
	uint8_t rssi2complLSB;
	int16_t rssi2compl;
	float rssiConverted;

	cc112x_read_register(CC112X_RSSI1, &rssi2complMSB);
	cc112x_read_register(CC112X_RSSI0, &rssi2complLSB);

	if (rssi2complLSB & 0x01)
	{
		// Shift the bits in place and add the 4 last bits
		rssi2compl = ((int8_t)(rssi2complMSB) << 4) | ((int8_t)(rssi2complLSB) >> 3);
		rssiConverted = (float)((rssi2compl)*0.0625) + RF_RSSI_OFFSET;
		return rssiConverted;
	}
	return 0;
}

uint8_t cc112x_get_partnum(void)
{
	uint8_t ver = 0;
	cc112x_read_register(CC112X_PARTVERSION, &ver);
	return ver;
}

/**
 * @fn          cc112x_get_bitrate
 *
 *@brief        Returns bitrate based on current register settings.
 *
 *
 * @param       none
 *
 *
 * @return      bitrate - the current bitrate
 */
uint32_t cc112x_get_symbol_rate(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	uint32_t drateM;
	uint32_t drateE;
	uint8_t readByte[3];
	float bitratetemp;
	uint32_t bitrate;
	//Read datarate from registers
	cc112x_read_register(CC112X_SYMBOL_RATE0, &readByte[0]);
	cc112x_read_register(CC112X_SYMBOL_RATE1, &readByte[1]);
	cc112x_read_register(CC112X_SYMBOL_RATE2, &readByte[2]);
	drateE = (readByte[2] & 0xF0) >> 4;
	drateM = (readByte[2] & 0x0F);
	drateM = drateM << 16;
	drateM |= (uint16_t)readByte[1] << 8;
	drateM |= readByte[0];
	bitratetemp = ((pow(2, 20) + drateM) * pow(2, drateE)) / (pow(2, 39)) * RF_XTAL_FREQ;
	bitrate = (uint32_t)roundf(bitratetemp);
	return bitrate;
}

void cc112x_set_symbol_rate_khz(uint8_t baud_rate_in_khz)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	srate_m_t drateM;

	symbol_rate2_t symbol_rate2;
	cc112x_read_register(CC112X_SYMBOL_RATE2, &symbol_rate2.reg);

	for (double_t tempE = 0; tempE < 16; tempE++)
	{
		double_t tempM = (baud_rate_in_khz * 1000.0 * pow(2, 39) / (pow(2, tempE) * RF_XTAL_FREQ)) - pow(2, 20) + .5;
		if (tempM < pow(2, 20))
		{
			symbol_rate2.srate_e = tempE;
			drateM.srate_m = tempM;
			break;
		}
	}
	ESP_LOGD(TAG, "Set Transmission Baud rate: %d k. E: %d, M: %x %x %x", baud_rate_in_khz, symbol_rate2.srate_e, drateM.srate_m_19_16, drateM.srate_m_15_8, drateM.srate_m_7_0);

	symbol_rate2.srate_m_19_16 = drateM.srate_m_19_16;

	cc112x_write_register(CC112X_SYMBOL_RATE0, drateM.srate_m_7_0);
	cc112x_write_register(CC112X_SYMBOL_RATE1, drateM.srate_m_15_8);
	cc112x_write_register(CC112X_SYMBOL_RATE2, symbol_rate2.reg);
}

bool cc112x_get_carrier_sense(void)
{
	rssi0_t rssi0;
	cc112x_read_register(CC112X_RSSI0, &rssi0.reg);
	return rssi0.carrier_sense;
}

void cc112x_set_agc_sync_behaviour(uint8_t value)
{
	agc_cfg1_t agc_cfg1;
	cc112x_read_register(CC112X_AGC_CFG1, &agc_cfg1.reg);
	agc_cfg1.agc_sync_behaviour = value > 7 ? 7 : value;
	cc112x_write_register(CC112X_AGC_CFG1, agc_cfg1.reg);
}

void cc112x_set_agc_win_size(uint8_t value)
{
	agc_cfg1_t agc_cfg1;
	cc112x_read_register(CC112X_AGC_CFG1, &agc_cfg1.reg);
	agc_cfg1.agc_win_size = value > 5 ? 5 : value;
	cc112x_write_register(CC112X_AGC_CFG1, agc_cfg1.reg);
}

void cc112x_set_agc_settle_wait(uint8_t value)
{
	agc_cfg1_t agc_cfg1;
	cc112x_read_register(CC112X_AGC_CFG1, &agc_cfg1.reg);
	agc_cfg1.agc_settle_wait = value > 3 ? 3 : value;
	cc112x_write_register(CC112X_AGC_CFG1, agc_cfg1.reg);
}

void cc112x_set_transparent_intfact(uint8_t value)
{
	mdmcfg0_t mdmcfg0;
	cc112x_read_register(CC112X_MDMCFG0, &mdmcfg0.reg);
	mdmcfg0.transparent_intfact = value > 2 ? 2 : value;
	cc112x_write_register(CC112X_MDMCFG0, mdmcfg0.reg);
}

void cc112x_set_modulation(mod_format_t mod)
{
	modcfg_dev_e_t modcfg_dev_e;
	cc112x_read_register(CC112X_MODCFG_DEV_E, &modcfg_dev_e.reg);
	modcfg_dev_e.mod_format = mod;
	cc112x_write_register(CC112X_MODCFG_DEV_E, modcfg_dev_e.reg);
}

void cc112x_set_ramp_shape(uint8_t value)
{
	pa_cfg1_t pa_cfg1;
	cc112x_read_register(CC112X_PA_CFG1, &pa_cfg1.reg);
	pa_cfg1.ramp_shape = value > 3 ? 3 : value;
	cc112x_write_register(CC112X_PA_CFG1, pa_cfg1.reg);
}

void cc112x_set_pa_ramping(bool value)
{
	pa_cfg2_t pa_cfg2;
	cc112x_read_register(CC112X_PA_CFG2, &pa_cfg2.reg);
	pa_cfg2.pa_cfg_reserved6 = value;
	cc112x_write_register(CC112X_PA_CFG2, pa_cfg2.reg);
}

void cc112x_set_agc_ask_decay(uint8_t value)
{
	agc_cfg0_t agc_cfg0;
	cc112x_read_register(CC112X_AGC_CFG0, &agc_cfg0.reg);
	agc_cfg0.agc_ask_decay = value > 3 ? 3 : value;
	cc112x_write_register(CC112X_AGC_CFG0, agc_cfg0.reg);
}

void cc112x_set_tx_power(int8_t power)
{
	if (power > 0x27)
		power = 0x27;
	else if (power < -16)
		power = -16;
	uint8_t pa_power_ramp;
	pa_power_ramp = 2 * power + 35;
	cc112x_set_pa_power_ramp(pa_power_ramp);
}

void cc112x_set_pa_power_ramp(uint8_t value)
{
	pa_cfg2_t pa_cfg2;
	ESP_ERROR_CHECK(cc112x_read_register(CC112X_PA_CFG2, &pa_cfg2.reg));
	ESP_LOGE(TAG, "%d", pa_cfg2.pa_power_ramp);
	pa_cfg2.pa_power_ramp = value > 0x3F ? 0x3F : value;
	ESP_ERROR_CHECK(cc112x_write_register(CC112X_PA_CFG2, pa_cfg2.reg));
}

void cc112x_set_ask_depth(uint8_t value)
{
	pa_cfg0_t pa_cfg0;
	ESP_ERROR_CHECK(cc112x_read_register(CC112X_PA_CFG0, &pa_cfg0.reg));
	ESP_LOGE(TAG, "%d", pa_cfg0.ask_depth);
	pa_cfg0.ask_depth = value > 0x0F ? 0x0F : value;
	ESP_ERROR_CHECK(cc112x_write_register(CC112X_PA_CFG0, pa_cfg0.reg));
}

void cc112x_set_gpio0_inv(bool inv)
{
	iocfgx_t iocfg0;
	cc112x_read_register(CC112X_IOCFG0, &iocfg0.reg);
	iocfg0.gpiox_inv = inv;
	cc112x_write_register(CC112X_IOCFG0, iocfg0.reg);
}

void cc112x_set_gpio2_inv(bool inv)
{
	iocfgx_t iocfg2;
	cc112x_read_register(CC112X_IOCFG2, &iocfg2.reg);
	iocfg2.gpiox_inv = inv;
	cc112x_write_register(CC112X_IOCFG2, iocfg2.reg);
}

void cc112x_set_gpio3_inv(bool inv)
{
	iocfgx_t iocfg3;
	cc112x_read_register(CC112X_IOCFG3, &iocfg3.reg);
	iocfg3.gpiox_inv = inv;
	cc112x_write_register(CC112X_IOCFG3, iocfg3.reg);
}
void cc112x_set_gpio0_cfg(gpiox_cfg_t cfg)
{
	iocfgx_t iocfg0;
	cc112x_read_register(CC112X_IOCFG0, &iocfg0.reg);
	iocfg0.gpiox_cfg = cfg;
	cc112x_write_register(CC112X_IOCFG0, iocfg0.reg);
}

void cc112x_set_gpio2_cfg(gpiox_cfg_t cfg)
{
	iocfgx_t iocfg2;
	cc112x_read_register(CC112X_IOCFG2, &iocfg2.reg);
	iocfg2.gpiox_cfg = cfg;
	cc112x_write_register(CC112X_IOCFG2, iocfg2.reg);
}

void cc112x_set_gpio3_cfg(gpiox_cfg_t cfg)
{
	iocfgx_t iocfg3;
	cc112x_read_register(CC112X_IOCFG3, &iocfg3.reg);
	iocfg3.gpiox_cfg = cfg;
	cc112x_write_register(CC112X_IOCFG3, iocfg3.reg);
}

void cc112x_set_cs_threshold(int8_t cs_th)
{
	int8_t th_cal = cs_th - RF_RSSI_OFFSET;
	cc112x_write_register(CC112X_AGC_CS_THR, (uint8_t)th_cal);
}

int8_t cc112x_get_cs_threshold(void)
{
	int8_t cs_thr;
	cc112x_read_register(CC112X_AGC_CS_THR, (uint8_t *)&cs_thr);
	return cs_thr + RF_RSSI_OFFSET;
}

void cc112x_set_bw_filter_khz(uint8_t bw_filter_value_khz)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	chan_bw_t chan_bw;
	iqic_t iqic;
	uint8_t freq_if;

	cc112x_read_register(CC112X_CHAN_BW, &chan_bw.reg);
	cc112x_read_register(CC112X_IQIC, &iqic.reg);
	cc112x_read_register(CC112X_FREQ_IF_CFG, &freq_if);

	uint8_t adc_cic_decfact[] = {20, 32};
	uint8_t bb_cic_decfact_last = 0;
	uint8_t adc_cic_decfact_last = 0;

	double calc_bw_filter;
	double tmp_bw_filter = 255000;

	for (uint8_t adc = 0; adc <= 1; adc++)
	{
		for (uint8_t bb = (adc == 0 ? 25 : 15); bb > 0; bb--)
		{
			calc_bw_filter = RF_XTAL_FREQ / (8.0 * adc_cic_decfact[adc] * bb);

			if (calc_bw_filter >= (bw_filter_value_khz * 1000.0) && calc_bw_filter <= tmp_bw_filter)
			{
				tmp_bw_filter = calc_bw_filter;
				adc_cic_decfact_last = adc;
				bb_cic_decfact_last = bb;
			}
		}
	}

	double f_if = (RF_XTAL_FREQ * freq_if / (1 << 15));

	if (f_if > 2 * tmp_bw_filter && f_if + tmp_bw_filter / 2 <= 100000)
	{
		iqic.iqic_en = true;
	}
	else
	{
		iqic.iqic_en = false;
	}

	ESP_LOGD(TAG, "bw %d kHz adc %x bb %x iq %d BW closest %f", bw_filter_value_khz, adc_cic_decfact_last, bb_cic_decfact_last, iqic.iqic_en, tmp_bw_filter);

	chan_bw.adc_cic_decfact = adc_cic_decfact_last;
	chan_bw.bb_cic_decfact = bb_cic_decfact_last;
	cc112x_write_register(CC112X_IQIC, iqic.reg);
	cc112x_write_register(CC112X_CHAN_BW, chan_bw.reg);
}

uint32_t cc112x_get_carrier_freq(void)
{
	uint8_t freq_regs[3];
	uint32_t freq_regs_uint32;
	uint32_t freq;
	double f_vco;

	cc112x_read_burst_registers(CC112X_FREQ2, freq_regs, 3);

	freq_regs_uint32 = (freq_regs[0] << 16) | (freq_regs[1] << 8) | freq_regs[2];

	/* Divide by 2^16 */
	f_vco = (double)freq_regs_uint32 / 65536;

	/* Multiply by oscillator frequency */
	f_vco *= (double)RF_XTAL_FREQ;

	/* VCO frequency -> Radio frequency */
	freq = f_vco / RF_LO_DIVIDER;

	return freq;
}

/******************************************************************************
 * @fn          cc112x_set_carrier_freq
 *
 * @brief       Calculate the required frequency registers and send the using
 *              serial connection to the RF tranceiver.
 *
 * input parameters
 *
 * @param       freq   -  frequency word provided in resolution
 *
 * output parameters
 *
 * @return      void
 */
void cc112x_set_carrier_freq(uint32_t freq)
{
	uint8_t freq_regs[3];
	uint32_t freq_regs_uint32;
	float f_vco;

	/* Radio frequency -> VCO frequency */
	f_vco = freq * RF_LO_DIVIDER;

	/* Divide by oscillator frequency */
	f_vco /= RF_XTAL_FREQ;

	/* Multiply by 2^16 */
	f_vco *= 65536;

	/* Convert value into uint32 from float */
	freq_regs_uint32 = (uint32_t)f_vco;

	/* return the frequency word */
	freq_regs[2] = ((uint8_t *)&freq_regs_uint32)[0];
	freq_regs[1] = ((uint8_t *)&freq_regs_uint32)[1];
	freq_regs[0] = ((uint8_t *)&freq_regs_uint32)[2];

	/* write the frequency word to the transciever */
	cc112x_write_burst_registers(CC112X_FREQ2, freq_regs, 3);
}

int32_t cc112x_get_mixer_freq(void)
{
	int8_t freq_if;
	float f_if;

	cc112x_read_register(CC112X_FREQ_IF_CFG, (uint8_t *)&freq_if);

	f_if = (float)RF_XTAL_FREQ * freq_if * (1 / 32768);

	return (int32_t)f_if;
}

/******************************************************************************
 * @fn          cc112x_get_freq_error
 *
 * @brief       Estimate the frequency error from two complement coded
 *              data to frequency error in Hertz
 *
 * input parameters
 *
 * @param      void
 *
 * output parameters
 *
 * @return      freq_error     - 32 bit signed integer value representing
 *                                frequency error in Hertz
 *
 */
int32_t cc112x_get_freq_error(void)
{
	uint8_t reg[2] = {0, 0};
	cc112x_read_register(CC112X_FREQOFF_EST0, &reg[0]);
	cc112x_read_register(CC112X_FREQOFF_EST1, &reg[1]);
	uint16_t freq_reg_error = ((uint16_t)reg[1] << 8) | reg[0];
	return cc112x_calculate_freq_error_est(freq_reg_error);
}

/**
 * @fn          cc112x_manual_calibration
 *
 * @brief       calibrates radio according to CC112x errata
 *
 * @param       none
 *
 * @return      none
 */
void cc112x_manual_calibration(void)
{

	uint8_t original_fs_cal2;
	uint8_t calResults_for_vcdac_start_high[3];
	uint8_t calResults_for_vcdac_start_mid[3];
	uint8_t marcstate;
	uint8_t writeByte;

	// 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	cc112x_write_register(CC112X_FS_VCO2, writeByte);

	// 2) Start with high VCDAC (original VCDAC_START + 2):
	cc112x_read_register(CC112X_FS_CAL2, &original_fs_cal2);
	writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
	cc112x_write_register(CC112X_FS_CAL2, writeByte);

	// 3) Calibrate and wait for calibration to be done (radio back in IDLE state)
	cc112x_command_strobe(CC112X_SCAL);

	do
	{
		cc112x_read_register(CC112X_MARCSTATE, &marcstate);
	} while (marcstate != 0x41);

	// 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with high VCDAC_START value
	cc112x_read_register(CC112X_FS_VCO2, &calResults_for_vcdac_start_high[FS_VCO2_INDEX]);
	cc112x_read_register(CC112X_FS_VCO4, &calResults_for_vcdac_start_high[FS_VCO4_INDEX]);
	cc112x_read_register(CC112X_FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX]);

	// 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	cc112x_write_register(CC112X_FS_VCO2, writeByte);

	// 6) Continue with mid VCDAC (original VCDAC_START):
	writeByte = original_fs_cal2;
	cc112x_write_register(CC112X_FS_CAL2, writeByte);

	// 7) Calibrate and wait for calibration to be done (radio back in IDLE state)
	cc112x_command_strobe(CC112X_SCAL);

	do
	{
		cc112x_read_register(CC112X_MARCSTATE, &marcstate);
	} while (marcstate != 0x41);

	// 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with mid VCDAC_START value
	cc112x_read_register(CC112X_FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX]);
	cc112x_read_register(CC112X_FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX]);
	cc112x_read_register(CC112X_FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX]);

	// 9) Write back highest FS_VCO2 and corresponding FS_VCO and FS_CHP result
	if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] > calResults_for_vcdac_start_mid[FS_VCO2_INDEX])
	{
		writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
		cc112x_write_register(CC112X_FS_VCO2, writeByte);
		writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
		cc112x_write_register(CC112X_FS_VCO4, writeByte);
		writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
		cc112x_write_register(CC112X_FS_CHP, writeByte);
	}
	else
	{
		writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
		cc112x_write_register(CC112X_FS_VCO2, writeByte);
		writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
		cc112x_write_register(CC112X_FS_VCO4, writeByte);
		writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
		cc112x_write_register(CC112X_FS_CHP, writeByte);
	}
}

void cc112x_set_unmodulated(bool value)
{
	cfm_data_cfg_t cfm_data_cfg;
	modcfg_dev_e_t modcfg_dev_e;
	pa_cfg2_t pa_cfg2;
	cc112x_read_register(CC112X_CFM_DATA_CFG, &cfm_data_cfg.reg);
	cc112x_read_register(CC112X_MODCFG_DEV_E, &modcfg_dev_e.reg);
	cc112x_read_register(CC112X_PA_CFG2, &pa_cfg2.reg);

	if (value == true && modcfg_dev_e.mod_format == MOD_ASK_OOK)
	{
		pa_cfg2.pa_cfg_reserved6 = 0;
		modcfg_dev_e.mod_format = MOD_2FSK;
		cfm_data_cfg.cfm_data_en = 1;
	}
	else if (value == false && modcfg_dev_e.mod_format == MOD_2FSK)
	{
		pa_cfg2.pa_cfg_reserved6 = 1;
		modcfg_dev_e.mod_format = MOD_ASK_OOK;
		cfm_data_cfg.cfm_data_en = 0;
	}

	cc112x_write_register(CC112X_CFM_DATA_CFG, cfm_data_cfg.reg);
	cc112x_write_register(CC112X_MODCFG_DEV_E, modcfg_dev_e.reg);
	cc112x_write_register(CC112X_PA_CFG2, pa_cfg2.reg);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/******************************************************************************
 * @fn          calculate_freq_error_est
 *
 * @brief       Convert the frequency error estimate from two complement coded
 *              data to frequency error in Hertz
 *
 * input parameters
 *
 * @param       freq_reg_error -  two complement formatted data from tranceiver
 *
 * output parameters
 *
 * @return      freq_error     - 32 bit signed integer value representing
 frequency error in Hertz
 */
static int32_t cc112x_calculate_freq_error_est(uint16_t freq_reg_error)
{
	uint32_t freq_error_est;
	int32_t freq_error_est_int;
	int8_t sign;

	/* the incoming data is 16 bit two complement format, separate "sign" */
	if (freq_reg_error > INT16_MAX)
	{
		freq_error_est = -(freq_reg_error - UINT16_MAX);
		sign = -1;
	}
	else
	{
		freq_error_est = freq_reg_error;
		sign = 1;
	}

	/* convert the data to hertz format in two steps to avoid integer overuns */
	freq_error_est = (freq_error_est * (RF_XTAL_FREQ / RF_LO_DIVIDER)) >> 8;
	freq_error_est = (freq_error_est * 1000) >> 10;

	/* re-assign the "sign" */
	freq_error_est_int = freq_error_est * sign;

	return freq_error_est_int;
}


/*
 * cc112x.h
 *
 *  Created on: 18 kwi 2020
 *      Author: kurza
 */

#ifndef CC112X_H
#define CC112X_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include "cc112x_defs.h"
#include "cc112x_regs.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void cc112x_init_config(void);
void cc112x_read_config(void);
void cc112x_print_config(void);

void cc112x_reset(void);
void cc112x_set_tx_state(void);
void cc112x_set_rx_state(void);
bool cc112x_get_rssi_valid(void);
void cc112x_set_idle_state(void);
int8_t cc112x_get_rssi(void);
uint8_t cc112x_get_partnum(void);
uint32_t cc112x_get_symbol_rate(void);
void cc112x_set_symbol_rate_khz(uint8_t baud_rate_in_khz);
state_t cc112x_get_chip_status(void);
int8_t cc112x_get_converted_rssi(void);
bool cc112x_get_carrier_sense(void);
void cc112x_set_agc_behaviour(uint8_t value);
void cc112x_set_agc_settle_wait(uint8_t value);
void cc112x_set_agc_ask_decay(uint8_t value);
void cc112x_set_ask_depth(uint8_t value);

void cc112x_set_transparent_intfact(uint8_t value);
void cc112x_set_modulation(mod_format_t mod);
void cc112x_set_ramp_shape(uint8_t value);
void cc112x_set_pa_ramping(bool value);
void cc112x_set_pa_power_ramp(uint8_t value);

void cc112x_set_gpio0_inv(bool inv);
void cc112x_set_gpio2_inv(bool inv);
void cc112x_set_gpio3_inv(bool inv);
void cc112x_set_gpio0_cfg(gpiox_cfg_t cfg);
void cc112x_set_gpio2_cfg(gpiox_cfg_t cfg);
void cc112x_set_gpio3_cfg(gpiox_cfg_t cfg);

uint32_t cc112x_get_carrier_freq(void);
void cc112x_set_carrier_freq(uint32_t freq);
int32_t cc112x_get_mixer_freq(void);

int32_t cc112x_get_freq_error(void);
void cc112x_set_cs_threshold(int8_t cs_th);
void cc112x_set_bw_filter_khz(uint8_t bw_filter_value_khz);
void cc112x_manual_calibration(void);
void cc112x_set_unmodulated(bool value);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CC112X_H */

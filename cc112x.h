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
#define CC112X_DEFAULT_CONFIG                          \
    {                                                  \
        .spi_host = CONFIG_CC112X_SPI_NUM,             \
        .spi_clock_speed_hz = CONFIG_CC112X_SPI_CLOCK, \
        .mosi = CONFIG_CC112X_MOSI_GPIO,               \
        .miso = CONFIG_CC112X_MISO_GPIO,               \
        .sclk = CONFIG_CC112X_CLK_GPIO,                \
        .cs = CONFIG_CC112X_CS_GPIO,                   \
        .hgm = CONFIG_CC112X_HGM_GPIO,                 \
    };

    /**********************
 *      TYPEDEFS
 **********************/

    /**********************
 * GLOBAL PROTOTYPES
 **********************/
    void cc112x_init_config(void);
    void cc112x_choice_config(cc112x_radio_cfg_t choice);
    void cc112x_set_radio_config(data_rate_t cfg);
    void cc112x_read_config(void);
    void cc112x_print_config(void);

    void cc112x_send_data(const uint8_t *buff, uint8_t len);

    void cc112x_reset(void);
    void cc112x_set_tx_state(void);
    void cc112x_set_flush_tx(void);
    void cc112x_set_rx_state(void);
    void cc112x_set_flush_rx(void);
    bool cc112x_get_rssi_valid(void);
    void cc112x_set_idle_state(void);
    int8_t cc112x_get_8bit_rssi(void);
    float cc112x_get_12bit_rssi(void);
    uint8_t cc112x_get_partnum(void);
    uint32_t cc112x_get_symbol_rate(void);
    void cc112x_set_symbol_rate_khz(uint8_t baud_rate_in_khz);
    state_t cc112x_get_chip_status(void);
    bool cc112x_get_carrier_sense(void);
    void cc112x_set_agc_sync_behaviour(uint8_t value);
    void cc112x_set_agc_win_size(uint8_t value);
    void cc112x_set_agc_settle_wait(uint8_t value);
    void cc112x_set_agc_ask_decay(uint8_t value);
    void cc112x_set_agc_hyst_level(uint8_t value);
    void cc112x_set_ask_depth(uint8_t value);

    void cc112x_set_tx_power(int8_t power);
    void cc112x_set_ask_min_power(uint8_t min_value);

    void cc112x_set_transparent_intfact(uint8_t value);
    void cc112x_set_modulation(mod_format_t mod);
    void cc112x_set_ramp_shape(uint8_t value);
    void cc112x_set_pa_ramping(bool value);
    void cc112x_set_pa_power_ramp(uint8_t value);
    uint8_t cc112x_get_pa_power_ramp(void);

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
    int8_t cc112x_get_cs_threshold(void);
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

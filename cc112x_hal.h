/*
 * cc112x_hal.h
 *
 *  Created on: 21 maj 2020
 *      Author: kurza
 */

#ifndef CC112X_HAL_H
#define CC112X_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <driver/spi_master.h>
#include <driver/gpio.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
	spi_host_device_t spi_host;
	gpio_num_t mosi;
	gpio_num_t miso;
	gpio_num_t sclk;
	gpio_num_t cs;
	uint32_t spi_clock_speed_hz;
	gpio_num_t hgm;
} cc112x_cfg_t;

typedef void (*carrier_sense_callback_t)(bool value);

typedef void *cc112x_handle_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
cc112x_handle_t cc112x_create(const cc112x_cfg_t *spi_cfg);
esp_err_t cc112x_destroy(cc112x_handle_t, bool);

esp_err_t cc112x_cs_intr_init(gpio_num_t cs_gpio, carrier_sense_callback_t cb);

esp_err_t cc112x_hgm_on(void);
esp_err_t cc112x_hgm_off(void);

esp_err_t cc112x_write_register(uint16_t addr, const uint8_t byte);
esp_err_t cc112x_write_burst_registers(uint16_t addr, const uint8_t *bytes, const uint8_t length);
esp_err_t cc112x_read_register(uint16_t addr, uint8_t *byte);
esp_err_t cc112x_read_burst_registers(uint16_t addr, uint8_t *bytes, uint8_t length);
esp_err_t cc112x_read_status( uint8_t *byte);
esp_err_t cc112x_command_strobe(uint8_t strobe);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CC112X_HAL_H */

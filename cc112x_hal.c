/*
 * cc112x_hal.c
 *
 *  Created on: 21 maj 2020
 *      Author: kurza
 */

/*********************
 *      INCLUDES
 *********************/
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "esp_log.h"
#include "esp_err.h"

#include "spi_master_ext.h"
#include "cc112x_hal.h"
#include "cc112x_defs.h"

/*********************
 *      DEFINES
 *********************/
#define ESP_INTR_FLAG_DEFAULT 0

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
	spi_device_handle_t spi_handle;
	cc112x_cfg_t spi_config;
} cc112x_dev_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void IRAM_ATTR cc112x_cs_gpio_isr_handler(void *arg);
static esp_err_t cc112x_hgm_init(gpio_num_t hgm_gpio);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "cc112x_hal";

static spi_device_handle_t cc112x_spi_handle = NULL;
static gpio_num_t cc112x_hgm_input = GPIO_NUM_NC;
static carrier_sense_callback_t carrier_sense_callback = NULL;

/**********************
 *      MACROS
 **********************/
#define CHECK(a, ret_val, str, ...)                                               \
      if (!(a))                                                                   \
      {                                                                           \
            ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            return (ret_val);                                                     \
      }

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
cc112x_handle_t cc112x_create(const cc112x_cfg_t *spi_cfg)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc112x_dev_t *cc112x_dev = (cc112x_dev_t*) calloc(1, sizeof(cc112x_dev_t));
	CHECK((cc112x_dev != NULL), NULL, "CC112X ALLOC FAIL");

	spi_bus_config_t buscfg =
		{
			.miso_io_num = spi_cfg->miso,
			.mosi_io_num = spi_cfg->mosi,
			.sclk_io_num = spi_cfg->sclk,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.intr_flags = ESP_INTR_FLAG_IRAM,
		};

	spi_device_interface_config_t devcfg =
		{
			.clock_speed_hz = spi_cfg->spi_clock_speed_hz, //Clock out at 10 MHz
			.mode = 3,                                     	//SPI mode 1
			.spics_io_num = spi_cfg->cs,                   	//CS pin
			.queue_size = 1, //We want to be able to queue 7 transactions at a time
			.command_bits = 2,
			.address_bits = 6,
		};

	esp_err_t error = ESP_OK;

	if (spi_handle[spi_cfg->spi_host] == NULL)
	{
		//Initialize the SPI bus
		ESP_LOGD(TAG, "Initialization SPI%d", spi_cfg->spi_host + 1);
		error = spi_bus_initialize(spi_cfg->spi_host, &buscfg, 0);
		CHECK((error == ESP_OK), NULL, "SPI device %d initialize fail", spi_cfg->spi_host);
	}

	//Attach the Device to the SPI bus
	error = spi_bus_add_device(spi_cfg->spi_host, &devcfg, &cc112x_spi_handle);
	CHECK((error == ESP_OK), NULL, "SPI device %d add fail", spi_cfg->spi_host);

	error = cc112x_hgm_init(spi_cfg->hgm);
	CHECK((error == ESP_OK), NULL, "HGM GPIO%d init fail", spi_cfg->hgm);

	cc112x_dev->spi_config = *spi_cfg;
	cc112x_dev->spi_handle = cc112x_spi_handle;

	if (spi_handle[spi_cfg->spi_host] == NULL)
	{
		spi_handle[spi_cfg->spi_host] = cc112x_spi_handle;
	}

	return cc112x_dev;
}

esp_err_t cc112x_cs_intr_init(gpio_num_t cs_gpio, carrier_sense_callback_t cb)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((cb != NULL), ESP_ERR_INVALID_ARG, "CALLBACK FUNC NOT EXIST");
	CHECK(GPIO_IS_VALID_GPIO(cs_gpio), ESP_ERR_INVALID_ARG, "WRONG GPIO NUM");

    gpio_config_t gpio_in_conf =
	{
		.mode = GPIO_MODE_INPUT,
		.intr_type = GPIO_INTR_ANYEDGE,
		.pin_bit_mask = (1ULL << cs_gpio),
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE, };

    esp_err_t ret = gpio_config(&gpio_in_conf);
    CHECK(ret == ESP_OK, ret, "GPIO %d CONFIG FAIL", cs_gpio);
    //install gpio isr service
    ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //CHECK(ret == ESP_OK, ret, "GPIO %d ISR SERVICE FAIL", w_up_gpio);
    //hook isr handler for specific gpio pin
    ret = gpio_isr_handler_add(cs_gpio, cc112x_cs_gpio_isr_handler, (void*) cs_gpio);
    CHECK(ret == ESP_OK, ret, "GPIO %d ISR HANDLER ADD FAIL", cs_gpio);
	carrier_sense_callback = cb;

	return ESP_OK;
}

esp_err_t cc112x_destroy(cc112x_handle_t handle, bool del_bus)
{
	cc112x_dev_t *dev = (cc112x_dev_t*) handle;
	esp_err_t error = ESP_OK;
	CHECK((handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");
	error = spi_bus_remove_device(dev->spi_handle);
	CHECK((error == ESP_OK), ESP_ERR_INVALID_STATE, "SPI device %d. remove fail", dev->spi_config.spi_host);
	if (del_bus)
	{
		error = spi_bus_free(dev->spi_config.spi_host);
		CHECK((error == ESP_OK), ESP_ERR_INVALID_STATE, "SPI device %d. free fail", dev->spi_config.spi_host);
	}
	free(handle);
	return error;
}

esp_err_t cc112x_hgm_on(void)
{
	CHECK((cc112x_hgm_input != GPIO_NUM_NC), ESP_ERR_INVALID_ARG, "HGM NOT INIT");
	esp_err_t resp = gpio_set_level(cc112x_hgm_input, true);
	return resp;
}

esp_err_t cc112x_hgm_off(void)
{
	CHECK((cc112x_hgm_input != GPIO_NUM_NC), ESP_ERR_INVALID_ARG, "HGM NOT INIT");
	esp_err_t resp = gpio_set_level(cc112x_hgm_input, false);
	return resp;
}

esp_err_t cc112x_write_register(uint16_t addr, const uint8_t byte)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((cc112x_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
		{
			.cmd = CC112X_WRITE_SINGLE,
			.addr = addr,
			.length = 8,
			.tx_buffer = &byte,
		};

	esp_err_t error = ESP_OK;
	if ((addr & 0xFF00) != 0) // send data with extended address in command field
	{
		tx_trans.flags |= (SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR);
		spi_transaction_ext_t tx_trans_ext =
			{
				.base = tx_trans,
				.command_bits = 2,
				.address_bits = 14
			};
		error = spi_device_polling_transmit(cc112x_spi_handle, (spi_transaction_t*) &tx_trans_ext);
	}
	else
	{
		error = spi_device_polling_transmit(cc112x_spi_handle, &tx_trans);
	}

	return error;
}

esp_err_t cc112x_write_burst_registers(uint16_t addr, const uint8_t *bytes, const uint8_t length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((cc112x_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
		{
			.cmd = (CC112X_WRITE_SINGLE | CC112X_BURST),
			.addr = addr,
			.length = length * 8,
			.tx_buffer = bytes,
		};

	esp_err_t error = ESP_OK;
	if ((addr & 0xFF00) != 0) // send data with extended address in command field
	{
		tx_trans.flags |= (SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR);
		spi_transaction_ext_t tx_trans_ext =
			{
				.base = tx_trans,
				.command_bits = 2,
				.address_bits = 14,
			};
		error = spi_device_polling_transmit(cc112x_spi_handle, (spi_transaction_t*) &tx_trans_ext);
	}
	else
	{
		error = spi_device_polling_transmit(cc112x_spi_handle, &tx_trans);
	}

	return error;
}

esp_err_t cc112x_read_register(uint16_t addr, uint8_t *byte)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((cc112x_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
		{
			.cmd = CC112X_READ_SINGLE,
			.addr = addr,
			.length = 8,
			.rx_buffer = byte,
		};

	esp_err_t error = ESP_OK;
	if ((addr & 0xFF00) != 0) // send data with extended address in command field
	{
		tx_trans.flags |= (SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR);
		spi_transaction_ext_t tx_trans_ext =
			{
				.base = tx_trans,
				.command_bits = 2,
				.address_bits = 14,
			};
		error = spi_device_polling_transmit(cc112x_spi_handle, (spi_transaction_t*) &tx_trans_ext);
	}
	else
	{
		error = spi_device_polling_transmit(cc112x_spi_handle, &tx_trans);
	}

	return error;
}

esp_err_t cc112x_read_burst_registers(uint16_t addr, uint8_t *bytes, uint8_t length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((cc112x_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
		{
			.cmd = (CC112X_READ_SINGLE | CC112X_BURST),
			.addr = addr,
			.length = 8 * length,
			.rx_buffer = bytes,
		};

	esp_err_t error = ESP_OK;
	if ((addr & 0xFF00) != 0) // send data with extended address in command field
	{
		tx_trans.flags |= (SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR);
		spi_transaction_ext_t tx_trans_ext =
			{
				.base = tx_trans,
				.command_bits = 2,
				.address_bits = 14,
			};
		error = spi_device_polling_transmit(cc112x_spi_handle, (spi_transaction_t*) &tx_trans_ext);
	}
	else
	{
		error = spi_device_polling_transmit(cc112x_spi_handle, &tx_trans);
	}

	return error;
}

esp_err_t cc112x_read_status(uint8_t *byte)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((cc112x_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	spi_transaction_t tx_trans =
		{
			.cmd = CC112X_WRITE_SINGLE,
			.addr = CC112X_SNOP,
			.length = 8,
			.rx_buffer = byte,
		};

	esp_err_t error = spi_device_polling_transmit(cc112x_spi_handle, &tx_trans);
	return error;
}

esp_err_t cc112x_command_strobe(uint8_t strobe)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((cc112x_spi_handle != NULL), ESP_ERR_INVALID_STATE, "SPI interface uninitialized.");

	uint8_t temp = 0;
	spi_transaction_t tx_trans =
		{
			.cmd = CC112X_WRITE_SINGLE,
			.addr = strobe,
			.length = 8,
			.rx_buffer = &temp,
		};

	esp_err_t error = spi_device_polling_transmit(cc112x_spi_handle, &tx_trans);
	return error;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static esp_err_t cc112x_hgm_init(gpio_num_t hgm_gpio)
{
	CHECK(GPIO_IS_VALID_GPIO(hgm_gpio), ESP_ERR_INVALID_ARG, "WRONG GPIO NUM");

	gpio_config_t gpio_hgm_conf =
		{
			.mode = GPIO_MODE_OUTPUT,
			.intr_type = GPIO_INTR_DISABLE,
			.pin_bit_mask = (1ULL << hgm_gpio),
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.pull_up_en = GPIO_PULLUP_DISABLE,
		};

	esp_err_t resp = gpio_config(&gpio_hgm_conf);
	CHECK((resp == ESP_OK), ESP_ERR_INVALID_ARG, "HGM GPIO CONFIG FAIL");
	cc112x_hgm_input = hgm_gpio;
	return resp;
}

static void IRAM_ATTR cc112x_cs_gpio_isr_handler(void *arg)
{
	gpio_num_t cs_gpio = (gpio_num_t)arg;
	int gpio_lvl = gpio_get_level(cs_gpio);
//	ESP_EARLY_LOGD(TAG, "%s - lvl %d", __FUNCTION__, gpio_lvl);
	if (carrier_sense_callback != NULL){
		carrier_sense_callback(gpio_lvl);
	}
}


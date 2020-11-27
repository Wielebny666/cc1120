/*
 * cc112x_regs.h
 *
 *  Created on: 13 maj 2020
 *      Author: kurzawa.p
 */

#ifndef CC112X_REGS_H
#define CC112X_REGS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include "cc112x_enums.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		uint8_t srate_m_7_0;
		uint8_t srate_m_15_8;
		uint8_t srate_m_19_16 :4;
	};
	uint32_t srate_m;
}srate_m_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		uint8_t srate_e :4;
		uint8_t srate_m_19_16 :4;
	};
	uint8_t reg;
}symbol_rate2_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		uint8_t reserve_3_0 :3;
		state_t state :3;
		uint8_t chip_rdy :1;
	};
	uint8_t reg;
}chip_status_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		uint8_t iqic_imgch_level_thr :2;
		uint8_t iqic_blen :2;
		uint8_t iqic_blen_settle :2;
		uint8_t iqic_update_coeff_en :1;
		uint8_t iqic_en :1;
	};
	uint8_t reg;
}iqic_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		uint8_t bb_cic_decfact :6;
		uint8_t adc_cic_decfact :1;
		uint8_t chfilt_bypass :1;
	};
	uint8_t reg;
}chan_bw_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		uint8_t rssi_valid :1;
		uint8_t carrier_sense_valid :1;
		uint8_t carrier_sense :1;
		uint8_t rssi_3_0 :4;
		uint8_t rssi0_not_used :1;
	};
	uint8_t reg;
}rssi0_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		marcstate_e marc_state :5;
		uint8_t marc_2pin_state :2;
		uint8_t marcstate_not_used :1;
	};
	uint8_t reg;
}marcstate_t;


typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		uint8_t mdmcfg0_reserved_1_0 :2;
		uint8_t viterbi_en : 1;
		uint8_t data_filter_en : 1;
		transparent_infact_t transparent_intfact : 2;
		uint8_t transparent_mode_en : 1;
		uint8_t mdmcfg0_not_used : 1;
	};
	uint8_t reg;
}mdmcfg0_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		uint8_t agc_settle_wait : 2;
		uint8_t agc_win_size : 3;
		uint8_t agc_sync_behaviour : 3;
	};
	uint8_t reg;
}agc_cfg1_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct
	{
		uint8_t agc_ask_decay : 2;
		uint8_t rssi_valid_cnt : 2;
		uint8_t agc_slewrate_limit : 2;
		uint8_t agc_hyst_level : 2;
	};
	uint8_t reg;
}agc_cfg0_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct __attribute__((__packed__, aligned(1)))
	{
		gpiox_cfg_t gpiox_cfg :6;
		uint8_t gpiox_inv :1;
		uint8_t gpiox_atran :1;
	};
	uint8_t reg;
}iocfgx_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct __attribute__((__packed__, aligned(1)))
	{
		uint8_t dev_e :3;
		mod_format_t mod_format :3;
		uint8_t modem_mode :2;
	};
	uint8_t reg;
}modcfg_dev_e_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct __attribute__((__packed__, aligned(1)))
	{
		uint8_t pa_power_ramp :6;
		uint8_t pa_cfg_reserved6 :1;
		uint8_t pa_cfg2_not_used :1;
	};
	uint8_t reg;
}pa_cfg2_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct __attribute__((__packed__, aligned(1)))
	{
		uint8_t ramp_shape :2;
		uint8_t second_ipl :3;
		uint8_t first_ipl :3;
	};
	uint8_t reg;
}pa_cfg1_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct __attribute__((__packed__, aligned(1)))
	{
		uint8_t upsampler_p :3;
		uint8_t ask_depth :4;
		uint8_t pa_cfg0_not_used :1;
	};
	uint8_t reg;
}pa_cfg0_t;

typedef union __attribute__((__packed__, aligned(1)))
{
	struct __attribute__((__packed__, aligned(1)))
	{
		uint8_t cfm_data_en :1;
		uint8_t cfm_data_reserved4_1 :4;
		uint8_t symbol_map_cfg :2;
		uint8_t cfm_data_cfg_not_used :1;
	};
	uint8_t reg;
}cfm_data_cfg_t;


typedef union __attribute__((__packed__, aligned(1)))
{
	struct __attribute__((__packed__, aligned(1)))
	{
		uint8_t pkt_format :2;
		uint8_t cca_mode :3;
		uint8_t pkt_cfg2_reserved5 :1;
		uint8_t pkt_cfg2_not_used :2;
	};
	uint8_t reg;
}pkt_cfg2_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CC112X_REGS_H */

/*
 * codec.h
 *
 *	Configuration for the Cirrus Logic CS43131 audio codec integrated into Pocket Player
 *	Most of the time I'm happy go lucky,
 *	99.9%Pure Inc.
 *
 *
 *  Created on: Nov 21, 2025
 *      Author: a400
 */

#ifndef CODEC_H
#define CODEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ------ I2C Config ------ */

// ADR pin tied to GND, 0x30 (default) //
#define CODEC_I2C_ADR	(0x30u)
// Timeout for control-port operations (ms) //
#define CODEC_I2C_TIMEOUT	(10u)
// Wait after Reset: tPUD >= 1.5ms + margin (ms) //
#define CODEC_TPUD_DELAY 	(3u)
// RESET low pulse (ms) //
#define CODEC_RESET_PULSE	(1u)

/* ----------------------- */

/* ------ Register Address Book ------ */

// Global //
#define CS43131_REG_DEVID_AB	(0x10000u)
#define CS43131_REG_DEVID_CD	(0x10001u)
#define CS43131_REG_DEVID_E		(0x10002u)

#define CS43131_REG_MCLK_CTRL   (0x10006u)	/* System Clocking Control */

#define CS43131_REG_ASP_RATE	(0x1000Bu)	/* Serial Port Sample Rate */
#define CS43131_REG_ASP_SIZE	(0x1000Cu)	/* Serial Port Bit Size */

#define CS43131_REG_PAD			(0x1000Du)

#define CS43131_REG_PWR_DOWN	(0x20000u)	/* Power Down Control */

// Phase-Locked Loop (PLL) //
#define CS43131_REG_PLL_SET_1	(0x30001u)	/* PLL Start */
#define CS43131_REG_PLL_SET_2	(0x30002u)	/* PLL_DIV_FRAC_0 (LSB) */
#define CS43131_REG_PLL_SET_3	(0x30003u)	/* PLL_DIV_FRAC_1 */
#define CS43131_REG_PLL_SET_4	(0x30004u)	/* PLL_DIV_FRAC_2 (MSB) */
#define CS43131_REG_PLL_SET_5	(0x30005u)	/* PLL_DIV_INT */
#define CS43131_REG_PLL_SET_6	(0x30008u)	/* PLL_OUT_DIV */
#define CS43131_REG_PLL_SET_7	(0x3000Au)	/* PLL_CAL_RATIO */
#define CS43131_REG_PLL_SET_8	(0x3001Bu)	/* PLL_MODE */
#define CS43131_REG_PLL_SET_9	(0x40002u)	/* PLL_REF_PREDIV */

// Audio Serial Port (ASP) //
#define CS43131_REG_ASP_CLKOUT	(0x40004u)	/* CLKOUT Control */
#define CS43131_REG_ASP_NUM_1	(0x40010u)	/* ASP Numerator 1 */
#define CS43131_REG_ASP_NUM_2	(0x40011u)	/* ASP Numerator 2 */
#define CS43131_REG_ASP_DEN_1	(0x40012u)	/* ASP Denominator 1 */
#define CS43131_REG_ASP_DEN_2	(0x40013u)	/* ASP Denominator 2 */

#define CS43131_REG_ASP_LRCK__HT1	(0x40014u)	/* ASP LRCK High Time 1 */
#define CS43131_REG_ASP_LRCK__HT2	(0x40015u)	/* ASP LRCK High Time 2 */
#define CS43131_REG_ASP_LRCK__P1	(0x40016u)	/* ASP LRCK Period 1 */
#define CS43131_REG_ASP_LRCK__P2	(0x40017u)	/* ASP LRCK Period 2 */

#define CS43131_REG_ASP_CLK		(0x40018u)	/* ASP Clock Config */
#define CS43131_REG_ASP_FRM		(0x40019u)	/* ASP Frame Config */

#define CS43131_REG_ASP_CH1		(0x50000u)	/* ASP Channel 1 Location */
#define CS43131_REG_ASP_CH2		(0x50001u)	/* ASP Channel 2 Location */

#define CS43131_REG_ASP_CH1_EN	(0x5000Au)	/* ASP Channel 1 Size & Enable */
#define CS43131_REG_ASP_CH2_EN	(0x5000Bu)	/* ASP Channel 2 Size & Enable */

// Headphone & PCM //
#define CS43131_REG_HP_OUT_CTRL		(0x80000u)	/* HP Output Control 1 */

#define CS43131_REG_HP_POP1			(0x10010u)	/* Pop-Free Power-up Helper 1 */
#define CS43131_REG_HP_POP2			(0x80032u)	/* Pop-Free Power-up Helper 2 */
#define CS43131_REG_HP_POP3			(0x80046u)  /* Pop-Free Power-down Helper */

#define CS43131_REG_HP_DETECT		(0xD0000u)	/* Headphone Detect */
#define CS43131_REG_HP_STATUS		(0xD0001u)	/* Headphone Status */

#define CS43131_REG_PCM_FILT		(0x90000u)	/* PCM Filter Option */
#define CS43131_REG_PCM_VOL_B		(0x90001u)	/* PCM Volume B */
#define CS43131_REG_PCM_VOL_A		(0x90002u)	/* PCM Volume A */
#define CS43131_REG_PCM_PATH_SIG1	(0x90003u)	/* PCM Path Signal Control 1 */
#define CS43131_REG_PCM_PATH_SIG2	(0x90004u)	/* PCM Path Signal Control 2 */

#define CS43131_REG_CLASS_H			(0xB0000u)	/* Class H Control */

// Interrupt Status & Masks //
#define CS43131_REG_INT_STATUS_1	(0xF0000u)	/* Interrupt Status 1 */
#define CS43131_REG_INT_STATUS_2	(0xF0001u)	/* Interrupt Status 2 */
#define CS43131_REG_INT_STATUS_3	(0xF0002u)	/* Interrupt Status 3 */
#define CS43131_REG_INT_STATUS_4	(0xF0003u)	/* Interrupt Status 4 */
#define CS43131_REG_INT_STATUS_5	(0xF0004u)	/* Interrupt Status 5 */

#define CS43131_REG_INT_MASK_1		(0xF0010u)	/* Interrupt Mask 1 */
#define CS43131_REG_INT_MASK_2		(0xF0011u)	/* Interrupt Mask 2 */
#define CS43131_REG_INT_MASK_3		(0xF0012u)	/* Interrupt Mask 3 */
#define CS43131_REG_INT_MASK_4		(0xF0013u)	/* Interrupt Mask 4 */
#define CS43131_REG_INT_MASK_5		(0xF0014u)	/* Interrupt Mask 5 */

/* -------------------------------♫--- */

/* ------ Bit-field Helpers (Mask & Values) ------ */

/* Global */

// Device ID //
#define CS43131_DEVID_AB (0x43u) /* 0100 0011, AB*/
#define CS43131_DEVID_CD (0x13u) /* 0001 0011, CD*/
#define CS43131_DEVID_E  (0x10u) /* 0001 0000, E*/

// System Clocking Control (0x10006) //
#define CS43131_MCLK_INT			(1u << 2)	/* 0: 24.576 Mhz, 1: 22.5792 Mhz */

#define CS43131_MCLK_SRC_MASK		(0x3u << 0)		/* bits 1:0 */
#define CS43131_MCLK_SRC_DIRECT		(0x0u << 0)		/* Direct MCLK/XTAL */
#define CS43131_MCLK_SRC_PLL		(0x1u << 0)		/* PLL */
#define CS43131_MCLK_SRC_RCO		(0x2u << 0)		/* RCO */

// Serial Port Sample Rate (0x1000B), lower nibble //
#define CS43131_ASP_SPRATE_MASK		(0x0Fu)
#define CS43131_ASP_SPRATE_44K1		(0x1u)
#define CS43131_ASP_SPRATE_48K		(0x2u)
#define CS43131_ASP_SPRATE_96K		(0x4u)
#define CS43131_ASP_SPRATE_192K		(0x6u)

// Serial Port Sample Bit Size (0x1000C) //
#define CS43131_ASP_SPSIZE_MASK		(0x03u)			/* bits 1:0 */
#define CS43131_ASP_SPSIZE_32		(0x0u)
#define CS43131_ASP_SPSIZE_24		(0x1u)
#define CS43131_ASP_SPSIZE_16		(0x2u)

// Pad Interface Configuration (0x1000Du) //
#define CS43131_ASP_3ST				(1u << 0)		/* ASP clocks Hi-Z in Master Mode when set */
#define CS43131_XSP_3ST				(1u << 1)		/* XSP clocks Hi-Z in Master Mode when set */

// Power Down Control (0x20000), Bit 0 Reserved //
#define CS43131_PDN_XSP				(1u << 7)
#define CS43131_PDN_ASP				(1u << 6)
#define CS43131_PDN_DSDIF			(1u << 5)
#define CS43131_PDN_HP				(1u << 4)
#define CS43131_PDN_XTAL			(1u << 3)
#define CS43131_PDN_PLL				(1u << 2)
#define CS43131_PDN_CLKOUT			(1u << 1)

// PLL (0x30000) //
#define CS43131_PLL_START			(1u << 0) 		/* PLL Setting 1 (0x30001) */
#define CS43131_PLL_MODE			(1u << 1) 		/* PLL Setting 8 (0x3001B) */

#define CS43131_PLL_REF_PREDIV_MASK		(0x03u)		/* PLL Setting 9 bits 1:0 */
#define CS43131_PLL_REF_PREDIV_1		(0x0u)
#define CS43131_PLL_REF_PREDIV_2		(0x1u)
#define CS43131_PLL_REF_PREDIV_4		(0x2u)
#define CS43131_PLL_REF_PREDIV_8		(0x3u)

// ASP Clock Configuration (0x40018) //
#define CS43131_ASP_MS				(1u << 4)		/* 0: Slave (default), 1: Master */
#define CS43131_ASP_SCPOL_OUT		(1u << 3)		/* SCLK output polarity */
#define CS43131_ASP_SCPOL_IN		(1u << 2)		/* SCLK input polarity (pad-to-logic) */
#define CS43131_ASP_LCPOL_OUT		(1u << 1)		/* LRCK output polarity */
#define CS43131_ASP_LCPOL_IN		(1u << 0)		/* LRCK input polarity (pad-to-logic) */

// ASP Frame Configuration (0x40019) //
#define CS43131_ASP_STP				(1u << 4)		/* Frame start phase */
#define CS43131_ASP_5050			(1u << 3)		/* 1: fixed 50% duty cycle (default) */

#define CS43131_ASP_FSD_MASK		(0x7u << 0)		/* bits 2:0 frame start delay */
#define CS43131_ASP_FSD_0_0			(0x0u << 0)
#define CS43131_ASP_FSD_0_5			(0x1u << 0)
#define CS43131_ASP_FSD_1_0			(0x2u << 0)		/* default; typical I2S */
#define CS43131_ASP_FSD_1_5			(0x3u << 0)
#define CS43131_ASP_FSD_2_0			(0x4u << 0)
#define CS43131_ASP_FSD_2_5			(0x5u << 0)

// ASP Channel N Size & Enable (0x5000A / 0x5000B) */
#define CS43131_ASP_CH_AP			(1u << 3)		/* Active phase (valid only is ASP_5050 = 1) */
#define CS43131_ASP_CH_EN			(1u << 2)		/* Channel enable */

#define CS43131_ASP_CH_RES_MASK		(0x3u << 0)		/* bits 1:0 */
#define CS43131_ASP_CH_RES_8		(0x0u << 0)
#define CS43131_ASP_CH_RES_16		(0x1u << 0)
#define CS43131_ASP_CH_RES_24		(0x2u << 0)		/* default */
#define CS43131_ASP_CH_RES_32		(0x3u << 0)

// Headphone Output Control (0x80000) //
#define CS43131_HP_CLAMPA			(1u << 7)
#define CS43131_HP_CLAMPB			(1u << 6)

#define CS43131_HP_OUT_FS_MASK		(0x3u << 4) 	/* bits 5:4 */
#define CS43131_HP_OUT_FS_0V5		(0x0u << 4)
#define CS43131_HP_OUT_FS_1V0		(0x1u << 4)
#define CS43131_HP_OUT_FS_1V41		(0x2u << 4)
#define CS43131_HP_OUT_FS_1V73		(0x3u << 4) 	/* default */

#define CS43131_HP_IN_EN			(1u << 3)
#define CS43131_HP_IN_LP			(1u << 2)
#define CS43131_HP_1DB_EN			(1u << 0)

// Headphone Pop-Free //
#define CS43131_HP_POP1				(0x99u)
#define CS43131_HP_POP_UP			(0x20u)
#define CS43131_HP_POP_DN			(0x21u)
#define CS43131_HP_POP_RST			(0x00u)

// Class H Control (0xB0000) //
#define CS43131_H_ADPT_PWR_MASK		(0x7u << 2)
#define CS43131_H_ADPT_PWR_SIGNAL	(0x7u << 2) 	/* 111: adapt to signal */
#define CS43131_H_HV_EN				(1u << 1)
#define CS43131_H_EXT_VCPFILT		(1u << 0)

// PCM Filter Option (0x90000) //
#define CS43131_PCM_FILTER_NOS			(1u << 5)
#define CS43131_PCM_FILTER_WBF_EN		(1u << 2)
#define CS43131_PCM_FILTER_HIGHPASS		(1u << 1)
#define CS43131_PCM_FILTER_DEEMP_ON		(1u << 0)

// PCM Path Signal Control 1 (0x90003) //
#define CS43131_PCM_PATH_RAMP_DOWN		(1u << 7)
#define CS43131_PCM_PATH_VOL_BEQA		(1u << 6)

#define CS43131_PCM_PATH_SZC_MASK		(0x3u << 4) /* soft ramp / zero-cross control */
#define CS43131_PCM_PATH_SZC_ASAP		(0x0u << 4) /* immediate */
#define CS43131_PCM_PATH_SZC_ZC			(0x1u << 4) /* zero cross change */
#define CS43131_PCM_PATH_SZC_SOFT		(0x2u << 4) /* soft ramp */
#define CS43131_PCM_PATH_SZC_SZC		(0x3u << 4) /* soft ramp and zero crossings */

#define CS43131_PCM_PATH_AMUTE			(1u << 3)
#define CS43131_PCM_PATH_AMUTEBEQA		(1u << 2)
#define CS43131_PCM_PATH_MUTE_A			(1u << 1)
#define CS43131_PCM_PATH_MUTE_B			(1u << 0)

// Interrupt Status 1 //
#define CS43131_INT1_PLL_READY_MASK	(1u << 2)
#define CS43131_INT1_PLL_READY			(1u << 2)

#define CS43131_INT1_PLL_ERR_MASK		(1u << 1)
#define CS43131_INT1_PLL_ERR			(1u << 1)

#define CS43131_INT1_PDN_DONE_MASK		(1u << 0)
#define CS43131_INT1_PDN_DONE			(1u << 0)

/* Timeouts + Interrupt Delays, ms */
#define CODEC_PLL_LOCK_TIMEOUT			(10u)
#define CODEC_PDN_DONE_TIMEOUT			(50u)
#define CODEC_POLL_STEP					(1u)

#define CODEC_MUTE_SETTLE				(150u)

// Interrupt Status 2 (ASP) //
#define CS43131_INT2_ASP_NOLRCK			(1u << 3)
#define CS43131_INT2_ASP_EARLY			(1u << 4)
#define CS43131_INT2_ASP_LATE			(1u << 5)
#define CS43131_INT2_ASP_ERROR			(1u << 6)
#define CS43131_INT2_ASP_OVFL			(1u << 7)

/* -------♫--------------------------- */

/* ------ Types ------ */

/*Status Codes*/
typedef enum {
	CODEC_OK = 0,
	CODEC_PARAM_ERROR,
	CODEC_I2C_ERROR,
	CODEC_ID_MISMATCH,
	CODEC_TIMEOUT,
	CODEC_PLL_ERROR
} codec_status_t;

/* Word length */
typedef enum {
	CODEC_WORD_LEN_16 = 16,
	CODEC_WORD_LEN_24 = 24,
	CODEC_WORD_LEN_32 = 32
} codec_word_len_t;

/* PCM SZC */
typedef enum {
	CODEC_PCM_SZC_IMM  = 0u,	// 00
	CODEC_PCM_SZC_ZC   = 1u,	// 01
	CODEC_PCM_SZC_SR   = 2u,	// 10
	CODEC_PCM_SZC_SRZC = 3u		// 11
} codec_pcm_szc_t;


/*Device Context*/
typedef struct {
	I2C_HandleTypeDef *hi2c;
	GPIO_TypeDef *reset_port;
	uint16_t reset_pin;
	uint8_t i2c_adr;
} codec_t;



/* ----------♫----------- */

/* ------ Control Plane API ------ */

// Codec I2C Comm. Start //
codec_status_t codec_init(codec_t *dev,
						  I2C_HandleTypeDef *hi2c,
						  GPIO_TypeDef *reset_port,
						  uint16_t reset_pin);

// Device ID check //

codec_status_t codec_id(codec_t *dev);

// 8-bit single-register read/write //

codec_status_t codec_reg_read8(codec_t *dev,
								uint32_t reg,
								uint8_t *value);

codec_status_t codec_reg_write8(codec_t *dev,
								uint32_t reg,
								uint8_t value);

// Reserved Bits Safety //
codec_status_t codec_reg_update(codec_t *dev,
								uint32_t reg,
								uint8_t mask,
								uint8_t value);

// PLL Configuration, 24Mhz ext.MCLK input -> 22.5792kHz int.MCLK //
codec_status_t codec_set_pll(codec_t *dev);

// ASP Configuration, Fs + word length//
codec_status_t codec_set_asp(codec_t *dev,
							 codec_word_len_t word_len);

// PCM Power Up - ASP + Headphone, pop-free //
codec_status_t codec_pcm_power_up(codec_t *dev);

// PCM Power Down //
codec_status_t codec_pcm_power_down(codec_t *dev);

// Mute Control //
codec_status_t codec_pcm_mute (codec_t *dev, bool mute);

// Volume Control //
codec_status_t codec_pcm_volume(codec_t *dev, uint8_t volume);

// Headphone Output + Amplifier Configuration //
codec_status_t codec_set_hp(codec_t *dev);

// PCM Path Configuration //
codec_status_t codec_pcm_path(codec_t *dev, codec_pcm_szc_t szc, bool auto_mute, bool vol_beqa);

// Codec Startup and Playback Prep  //
codec_status_t codec_music_prep (codec_t *dev, codec_word_len_t word_len, uint8_t volume);

// Start Music ♫ //
codec_status_t codec_music_play(codec_t *dev);

// Pause Playback (Mute + Settle) //
codec_status_t codec_music_end(codec_t *dev);

// Stop Playback //
codec_status_t codec_stop(codec_t *dev);

// End Playback + Shutdown Codec //
codec_status_t codec_shutdown(codec_t *dev);


#ifdef __cplusplus
}
#endif


#endif /* CODEC_H_ */

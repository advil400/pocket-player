/*
 * codec.c
 *
 *	Waking+Life Technologies
 *
 *  Created on: Nov 21, 2025
 *      Author: a400
 */

#include "codec.h"

/* ADR Shift for HAL compatibility */
static uint16_t codec_hal_adr(const codec_t *dev){
	return (uint16_t)(dev->i2c_adr << 1);
}

/* Register Access Routines*/

// Control Plane MAP //

static void codec_build_map(uint32_t reg, uint8_t map[3]){
	map[0] = (uint8_t)((reg >> 16) & 0xFFu); /* high [23:16] */
	map[1] = (uint8_t)((reg >> 8)  & 0xFFu); /* mid [15:8] */
	map[2] = (uint8_t)(reg		   & 0xFFu); /* low [7:0] */
}

// MAP Register Read //

codec_status_t codec_reg_read8(codec_t *dev,
							   uint32_t reg,
							   uint8_t *value)
{
	if (dev == NULL || dev->hi2c == NULL || value == NULL) {
		return CODEC_PARAM_ERROR;
	}

	uint8_t prep[4];
	uint8_t map[3];
	codec_build_map(reg, map);

	prep[0] = map[0];
	prep[1] = map[1];
	prep[2] = map[2];

	/* Single-byte read: SIZE=00 (8-bit), INCR=0 */
	prep [3] = 0x00u;

	HAL_StatusTypeDef hal =
			HAL_I2C_Master_Transmit(dev->hi2c,
									codec_hal_adr(dev),
									prep,
									sizeof(prep),
									CODEC_I2C_TIMEOUT);
	if (hal!= HAL_OK) {
		return CODEC_I2C_ERROR;
	}

	hal = HAL_I2C_Master_Receive(dev->hi2c,
								 codec_hal_adr(dev),
								 value,
								 1u,
								 CODEC_I2C_TIMEOUT);
	if (hal!= HAL_OK){
		return CODEC_I2C_ERROR;
	}
	return CODEC_OK;
}

// MAP Register Write //

codec_status_t codec_reg_write8(codec_t *dev,
								uint32_t reg,
								uint8_t value)
{
	if (dev == NULL || dev->hi2c == NULL){
		return CODEC_PARAM_ERROR;
	}

	uint8_t buf[5];
	uint8_t map[3];
	codec_build_map(reg, map);

	buf[0] = map [0];
	buf[1] = map [1];
	buf[2] = map [2];

	/* Single-byte write: SIZE=00 (8-bit), INCR=0 */
	buf[3] = 0x00u;
	buf[4] = value;

	HAL_StatusTypeDef hal =
			HAL_I2C_Master_Transmit(dev->hi2c,
									codec_hal_adr(dev),
									buf,
									(uint16_t)sizeof(buf),
									CODEC_I2C_TIMEOUT);
	if (hal != HAL_OK) {
		return CODEC_I2C_ERROR;
	}

	return CODEC_OK;
}

// Reserved-Bit Safe Update //

codec_status_t codec_reg_update(codec_t *dev,
								uint32_t reg,
								uint8_t mask,
								uint8_t value)
{
	if (dev == NULL || dev->hi2c == NULL){
		return CODEC_PARAM_ERROR;
	}

	uint8_t old;
	codec_status_t st;

	st = codec_reg_read8(dev, reg, &old);
	if (st != CODEC_OK) {
		return st;
	}

	uint8_t new = (uint8_t)((old & ~mask) | (value & mask));
	if (new == old) {
		return CODEC_OK;
	}

	return codec_reg_write8(dev, reg, new);
}

/* Public API */

// I2C Init & Reset //
codec_status_t codec_init(codec_t *dev,
						  I2C_HandleTypeDef *hi2c,
						  GPIO_TypeDef *reset_port,
						  uint16_t reset_pin)
{
	if (dev == NULL || hi2c == NULL || reset_port == NULL) {
		return CODEC_PARAM_ERROR;
	}

	dev->hi2c = hi2c;
	dev->reset_port = reset_port;
	dev->reset_pin = reset_pin;
	dev->i2c_adr = CODEC_I2C_ADR;

	/* Initial RESET */
	HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(CODEC_RESET_PULSE);
	HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_SET);
	HAL_Delay(CODEC_TPUD_DELAY);

	/* Initial I2C address check */
	HAL_StatusTypeDef hal =
			HAL_I2C_IsDeviceReady(dev->hi2c,
								  codec_hal_adr(dev),
								  3u,
								  CODEC_I2C_TIMEOUT);
	if (hal != HAL_OK) {
		return CODEC_I2C_ERROR;
	}

	/* Device ID Read Check */
	return codec_id(dev);
}

// Check CS43131 Device ID //
codec_status_t codec_id(codec_t *dev)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}

	uint8_t map_ctrl[4];
	uint8_t map[3];

	codec_build_map(CS43131_REG_DEVID_AB, map);

	map_ctrl[0] = map[0];
	map_ctrl[1] = map[1];
	map_ctrl[2] = map[2];

	/* Control Byte: SIZE=00(8-bit), INCR=1 (auto-increment multi-byte AB_CD_E read) */
	map_ctrl[3] = 0x01u;

	HAL_StatusTypeDef hal =
			HAL_I2C_Master_Transmit(dev->hi2c,
									codec_hal_adr(dev),
									map_ctrl,
									sizeof(map_ctrl),
									CODEC_I2C_TIMEOUT);
	if (hal != HAL_OK) {
		return CODEC_I2C_ERROR;
	}

	uint8_t id[3] = {0};
	hal = HAL_I2C_Master_Receive(dev->hi2c,
								 codec_hal_adr(dev),
								 id,
								 sizeof(id),
								 CODEC_I2C_TIMEOUT);
	if (hal != HAL_OK) {
		return CODEC_I2C_ERROR;
	}

	/* Check against datasheet defaults for Device ID A-E */
	if (id[0] != CS43131_DEVID_AB ||
		id[1] != CS43131_DEVID_CD ||
		id[2] != CS43131_DEVID_E) {
		return CODEC_ID_MISMATCH;
	}

	return CODEC_OK;

}

// Clear Interrupt Status Sticky Bits //
static codec_status_t codec_clear_sticky(codec_t *dev)
{
	uint8_t dint;
	codec_status_t st;

	st = codec_reg_read8(dev, CS43131_REG_INT_STATUS_1, &dint);
	if (st != CODEC_OK) return st;
	st = codec_reg_read8(dev, CS43131_REG_INT_STATUS_2, &dint);
	if (st != CODEC_OK) return st;
	st = codec_reg_read8(dev, CS43131_REG_INT_STATUS_3, &dint);
	if (st != CODEC_OK) return st;
	st = codec_reg_read8(dev, CS43131_REG_INT_STATUS_4, &dint);
	if (st != CODEC_OK) return st;
	st = codec_reg_read8(dev, CS43131_REG_INT_STATUS_5, &dint);
	if (st != CODEC_OK) return st;

	return CODEC_OK;
}

static codec_status_t codec_unmask_pdn_done(codec_t *dev)
{
	return codec_reg_update(dev,
							CS43131_REG_INT_MASK_1,
							CS43131_INT1_PDN_DONE_MASK,
							0x00u);
}

static codec_status_t codec_wait_pll(codec_t *dev, uint32_t wait)
{
	uint32_t t0 = HAL_GetTick();

	while ((HAL_GetTick() - t0) < wait){
		uint8_t int1 = 0;
		codec_status_t st = codec_reg_read8(dev, CS43131_REG_INT_STATUS_1, &int1);

		if (st != CODEC_OK) {
			return st;
		}

		if (int1 & CS43131_INT1_PLL_ERR) {
			return CODEC_PLL_ERROR;
		}

		if (int1 & CS43131_INT1_PLL_READY) {
			return CODEC_OK;
		}
		HAL_Delay(CODEC_POLL_STEP);
	}
	return CODEC_TIMEOUT;
}

static codec_status_t codec_wait_pdn(codec_t *dev, uint32_t wait)
{
	const uint32_t t0 = HAL_GetTick();

	while ((HAL_GetTick() - t0) < wait) {
		uint8_t int1 = 0;
		codec_status_t st = codec_reg_read8(dev, CS43131_REG_INT_STATUS_1, &int1);
		if (st != CODEC_OK) return st;

		if ((int1 & CS43131_INT1_PDN_DONE) != 0u){
			return CODEC_OK;
		}
		HAL_Delay(CODEC_POLL_STEP);
	}
	return CODEC_TIMEOUT;
}

// PLL Config 11.2896Mhz -> 22.5792Mhz + Start //
codec_status_t codec_set_pll (codec_t *dev)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}

	codec_status_t st;

	// 1. PLL inactive during config, PLL_START=0 //
	st = codec_reg_update(dev, CS43131_REG_PLL_SET_1, CS43131_PLL_START, 0x00u);
	if (st != CODEC_OK) return st;

	// 2. Clear sticky interrupt status 1 //
	st = codec_clear_sticky(dev);
	if (st != CODEC_OK) return st;

	/*
	     * Table 4-5, XIN=11.2896 MHz, PLL_OUT=22.5792 MHz:
	     *   PLL_REF_PREDIV  = divide by 4 (value 2)
	     *   PLL_DIV_INT     = 0x40
	     *   PLL_DIV_FRAC    = 0x000000
	     *   PLL_OUT_DIV     = 0x08
	     *   PLL_MODE        = 1
	     *   PLL_CAL_RATIO   = 128 (0x80)
	*/

	// 3. Set PLL Clocking Configuration //
	st = codec_reg_write8(dev, CS43131_REG_PLL_SET_2, 0x00u); if (st != CODEC_OK) return st; /* FRAC_0, 0 */
	st = codec_reg_write8(dev, CS43131_REG_PLL_SET_3, 0x00u); if (st != CODEC_OK) return st; /* FRAC_1, 0 */
	st = codec_reg_write8(dev, CS43131_REG_PLL_SET_4, 0x00u); if (st != CODEC_OK) return st; /* FRAC_2, 0 */

	st = codec_reg_write8(dev, CS43131_REG_PLL_SET_5, 0x40u); if (st != CODEC_OK) return st; /* Integer Feedback Divider, 0x40 */

	st = codec_reg_write8(dev, CS43131_REG_PLL_SET_6, 0x08u); if (st != CODEC_OK) return st; /* Output Divider, 0x08 */

	st = codec_reg_write8(dev, CS43131_REG_PLL_SET_7, 0x80u); if (st != CODEC_OK) return st; /* Calibration Ratio, 128 */

	/* PLL_MODE = 1 */
	st = codec_reg_update(dev,
						  CS43131_REG_PLL_SET_8,
						  CS43131_PLL_MODE,
						  CS43131_PLL_MODE);
	if (st != CODEC_OK) return st;

	/* PREDIV = /4 */
	st = codec_reg_update(dev,
						  CS43131_REG_PLL_SET_9,
						  CS43131_PLL_REF_PREDIV_MASK,
						  CS43131_PLL_REF_PREDIV_4);
	if (st != CODEC_OK) return st;

	// 4. Unmask PLL interrupt in INT_MASK_1 //
	st = codec_reg_update(dev,
						  CS43131_REG_INT_MASK_1,
						  (uint8_t)(CS43131_INT1_PLL_READY_MASK | CS43131_INT1_PLL_ERR_MASK),
						  0x00u);
	if (st != CODEC_OK) return st;

	// 5. Power-up PLL block //
	st = codec_reg_update(dev,
						  CS43131_REG_PWR_DOWN,
						  CS43131_PDN_PLL,
						  0x00u);
	if (st != CODEC_OK) return st;

	// 6. PLL Start //
	st = codec_reg_update(dev,
						  CS43131_REG_PLL_SET_1,
						  CS43131_PLL_START,
						  CS43131_PLL_START);
	if (st != CODEC_OK) return st;

	// 7. Wait for PLL ready (poll INT_STATUS_1) //
	st = codec_wait_pll(dev, CODEC_PLL_LOCK_TIMEOUT); if (st != CODEC_OK) return st;

	// 8. Switch internal MCLK to PLL, MCLK_INT = 22.5792Mhz //
	st = codec_reg_update(dev,
						  CS43131_REG_MCLK_CTRL,
						  (uint8_t)(CS43131_MCLK_INT | CS43131_MCLK_SRC_MASK),
						  (uint8_t)(CS43131_MCLK_INT | CS43131_MCLK_SRC_PLL));
	if (st != CODEC_OK) return st;

	// 9. Wait at least 150us after switching MCLK source, 1ms for now while μ-sec timer pending //
	HAL_Delay(1);

	return CODEC_OK;

}

// Audio Serial Port Configuration //
codec_status_t codec_set_asp(codec_t *dev, codec_word_len_t word_len)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}

	codec_status_t st;

	/* Set ASP_SPRATE = 44.1kHz default, dynamic later */
	st = codec_reg_update(dev,
						  CS43131_REG_ASP_RATE,
						  CS43131_ASP_SPRATE_MASK,
						  CS43131_ASP_SPRATE_44K1);
	if (st != CODEC_OK) return st;

	/* ASP_SPSIZE, 32-bit slots (STM32 16-bit extended) */

	st = codec_reg_update(dev,
						  CS43131_REG_ASP_SIZE,
						  CS43131_ASP_SPSIZE_MASK,
						  CS43131_ASP_SPSIZE_32);
	if (st != CODEC_OK) return st;

	/* Clock & Frame Configuration, I2S slave mode */
	st = codec_reg_write8(dev, CS43131_REG_ASP_CLK, 0x0Cu); // Clock //
	if (st != CODEC_OK) return st;

	st = codec_reg_write8(dev, CS43131_REG_ASP_FRM, 0x0Au); // Frame //
	if (st != CODEC_OK) return st;

	/* Channel locations (I2S 50/50 mode uses LRCK phase to select channel) */
	st = codec_reg_write8(dev, CS43131_REG_ASP_CH1, 0x00u);
	if (st != CODEC_OK) return st;

	st = codec_reg_write8(dev, CS43131_REG_ASP_CH2, 0x00u);
	if (st != CODEC_OK) return st;

	/* Channel resolution + enable */
	uint8_t res;
	switch(word_len){
		case CODEC_WORD_LEN_16: res = 0x01u; break;
		case CODEC_WORD_LEN_24: res = 0x02u; break;
		case CODEC_WORD_LEN_32: res = 0x03u; break;
		default: return CODEC_PARAM_ERROR;
	}

	uint8_t ch1 = (uint8_t)((0u << 3) | (1u << 2) | res); // AP=0, EN=1 //
	uint8_t ch2 = (uint8_t)((1u << 3) | (1u << 2) | res); // AP=1, EN=1 //

	st = codec_reg_write8(dev, CS43131_REG_ASP_CH1_EN, ch1);
	if (st != CODEC_OK) return st;

	st = codec_reg_write8(dev, CS43131_REG_ASP_CH2_EN, ch2);
	if (st != CODEC_OK) return st;

	return CODEC_OK;
}

// PCM Power-up (ASP + HP) //

codec_status_t codec_pcm_power_up(codec_t *dev)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}

	codec_status_t st;

	/*
	     * Example 5-5 (PCM Power-Up Sequence):
	     *   1) Pop-free power-up settings
	     *      0x10010 = 0x99
	     *      0x80032 = 0x20
	     *   2) Power up ASP (clear PDN_ASP)
	     *   3) Power up HP (clear PDN_HP)
	     *   4) Wait 12ms
	     *   5) Restore defaults
	*/

	/* Pop-free start */
	st = codec_reg_write8(dev, CS43131_REG_HP_POP1, 0x99u);
	if (st != CODEC_OK) return st;

	st = codec_reg_write8(dev, CS43131_REG_HP_POP2, 0x20u);
	if (st != CODEC_OK) return st;

	/* ASP Up */
	st = codec_reg_update(dev,
						  CS43131_REG_PWR_DOWN,
						  CS43131_PDN_ASP,
						  0x00u); /* PDN_ASP = 0 */
	if (st != CODEC_OK) return st;

	/* HP up */
	st = codec_reg_update(dev,
						  CS43131_REG_PWR_DOWN,
						  CS43131_PDN_HP,
						  0x00u); /* PDN_HP = 0 */
	if (st != CODEC_OK) return st;

	/* Analog dwell time */
	HAL_Delay(12);

	/* Pop-free restore */
	st = codec_reg_write8(dev, CS43131_REG_HP_POP2, 0x00u);
		if (st != CODEC_OK) return st;

	st = codec_reg_write8(dev, CS43131_REG_HP_POP1, 0x00u);
		if (st != CODEC_OK) return st;

	return CODEC_OK;

}

codec_status_t codec_pcm_power_down(codec_t *dev)
{
	if (dev == NULL || dev->hi2c == NULL) {
			return CODEC_PARAM_ERROR;
		}

	codec_status_t st;

	/* 1. Clear sticky */
	st = codec_clear_sticky(dev);
	if (st != CODEC_OK) return st;

	st = codec_unmask_pdn_done(dev);
	if (st != CODEC_OK) return st;

	/* 2. PDN_HP = 1 (power down amplifier) */
	st = codec_reg_update(dev,
						  CS43131_REG_PWR_DOWN,
						  CS43131_PDN_HP,
						  CS43131_PDN_HP);
	if (st != CODEC_OK) return st;

	/* 3. Wait for PDN_DONE */
	st = codec_wait_pdn(dev, CODEC_PDN_DONE_TIMEOUT);
	if (st != CODEC_OK) return st;

	/* 4. PDN_ASP = 1 (power down ASP) */
	st = codec_reg_update(dev,
						  CS43131_REG_PWR_DOWN,
						  CS43131_PDN_ASP,
						  CS43131_PDN_ASP);
	if (st != CODEC_OK) return st;

	return CODEC_OK;

}

// Mute Audio //
codec_status_t codec_pcm_mute(codec_t *dev, bool mute)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}

	uint8_t mask = (uint8_t)(CS43131_PCM_PATH_MUTE_A | CS43131_PCM_PATH_MUTE_B);
	uint8_t value = mute ? mask : 0x00u;

	return codec_reg_update(dev,
							CS43131_REG_PCM_PATH_SIG1,
							mask,
							value);
}

// PCM Path Config //
codec_status_t codec_pcm_path(codec_t *dev, codec_pcm_szc_t szc, bool auto_mute, bool vol_beqa)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}

	uint8_t mask = (uint8_t)(CS43131_PCM_PATH_SZC_MASK | CS43131_PCM_PATH_AMUTE | CS43131_PCM_PATH_AMUTEBEQA | CS43131_PCM_PATH_VOL_BEQA);

	uint8_t value = 0u;
	value |= (uint8_t)((uint8_t)szc << 4); // PCM_SZC[1:0] at bits 5:4
	if (auto_mute) {
		value |= (uint8_t)CS43131_PCM_PATH_AMUTE;
		value |= (uint8_t)CS43131_PCM_PATH_AMUTEBEQA;
	}
	if (vol_beqa) value |= (uint8_t)CS43131_PCM_PATH_VOL_BEQA;

	return codec_reg_update(dev, CS43131_REG_PCM_PATH_SIG1, mask, value);
}

// Headphone Output + Amplifier Configuration //
codec_status_t codec_set_hp(codec_t *dev)
{
	if (dev == NULL || dev->hi2c == NULL) {
			return CODEC_PARAM_ERROR;
		}

	uint8_t pdn = 0;
	codec_status_t st;

	st = codec_reg_read8(dev, CS43131_REG_PWR_DOWN, &pdn);
	if (st != CODEC_OK) return st;

	if ((pdn & CS43131_PDN_HP) == 0u){
		return CODEC_PARAM_ERROR;
	}

	// Class H: ADPT_PWR=111, HV_EN=0, EXT_VCPFILT=0  -> value = 0x1C
	st = codec_reg_update(dev,
						  CS43131_REG_CLASS_H,
						  (uint8_t)(CS43131_H_ADPT_PWR_MASK |
								    CS43131_H_HV_EN |
									CS43131_H_EXT_VCPFILT),
						  (uint8_t)(CS43131_H_ADPT_PWR_SIGNAL));
	if (st != CODEC_OK) return st;

	/* HP Output Control 1:
	 * - OUT_FS = 1.41Vrms (10)
	 * - HP_IN_EN = 0 (disable HPIN path)
	 * - HP_IN_LP = 0
	 * - +1dB_EN  = 0
	 * - CLAMP opt-out = 0 (keep clamping)
	 */
	const uint8_t hp_mask = (uint8_t)(CS43131_HP_CLAMPA | CS43131_HP_CLAMPB |
									  CS43131_HP_OUT_FS_MASK |
									  CS43131_HP_IN_EN | CS43131_HP_IN_LP |
									  CS43131_HP_1DB_EN);

	const uint8_t hp_value = (uint8_t)(CS43131_HP_OUT_FS_1V41);

	st = codec_reg_update(dev, CS43131_REG_HP_OUT_CTRL, hp_mask, hp_value);
	if (st != CODEC_OK) return st;

	return CODEC_OK;
}

// Volume Control //
codec_status_t codec_pcm_volume(codec_t *dev, uint8_t volume)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}

	if (volume == 0xFFu){
		return CODEC_PARAM_ERROR;
	}

	codec_status_t st = codec_reg_write8(dev, CS43131_REG_PCM_VOL_A, volume);
	if (st != CODEC_OK) return st;

	return codec_reg_write8(dev, CS43131_REG_PCM_VOL_B, volume);
}

// Prepare for Playback //
codec_status_t codec_music_prep(codec_t *dev, codec_word_len_t word_len, uint8_t volume)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}

	codec_status_t st;

	st = codec_pcm_mute(dev, true);		// start muted //
	if (st != CODEC_OK) return st;

	st = codec_set_pll(dev);
	if (st != CODEC_OK) return st;

	st = codec_set_asp(dev, word_len);
	if (st != CODEC_OK) return st;

	st = codec_pcm_path(dev, CODEC_PCM_SZC_SR, false, true);
	if (st != CODEC_OK) return st;

	st = codec_set_hp(dev);
	if (st != CODEC_OK) return st;

	st = codec_pcm_volume(dev, volume);
	if (st != CODEC_OK) return st;

	st = codec_pcm_power_up(dev);
	if (st != CODEC_OK) return st;

	st = codec_clear_sticky(dev);
	if (st != CODEC_OK) return st;

	return CODEC_OK;
}

// MUSIC!!! //
codec_status_t codec_music_play(codec_t *dev)
{
	return codec_pcm_mute(dev, false);
}

// End Playback (Mute + Settle)  //
codec_status_t codec_music_end(codec_t *dev)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}

	codec_status_t st = codec_pcm_mute(dev, true);
	if (st != CODEC_OK) return st;

	HAL_Delay(CODEC_MUTE_SETTLE);
	return CODEC_OK;

}

// Codec Shutdown //
codec_status_t codec_stop(codec_t *dev)
{
	if (dev == NULL || dev->hi2c == NULL) {
		return CODEC_PARAM_ERROR;
	}
	codec_status_t st;

	// Shutdown PCM //
	st = codec_pcm_power_down(dev);
	if (st != CODEC_OK) return st;

	// Switch MCLK source to RCO //
	st = codec_reg_update(dev,
						  CS43131_REG_MCLK_CTRL,
						  (uint8_t)(CS43131_MCLK_INT | CS43131_MCLK_SRC_MASK),
						  (uint8_t)(CS43131_MCLK_INT | CS43131_MCLK_SRC_RCO));
	if (st != CODEC_OK) return st;

	HAL_Delay(1);

	// Stop PLL output //
	st = codec_reg_update(dev, CS43131_REG_PLL_SET_1, CS43131_PLL_START, 0x00u);
	if (st != CODEC_OK) return st;

	// Power down PPL //
	st = codec_reg_update(dev, CS43131_REG_PWR_DOWN, CS43131_PDN_PLL, CS43131_PDN_PLL);
	if (st != CODEC_OK) return st;

	// Clear sticky bits //
	st = codec_clear_sticky(dev);
	if (st != CODEC_OK) return st;

	return CODEC_OK;
}

// End Playback + Shutdown Codec //
codec_status_t codec_shutdown(codec_t *dev)
{
	codec_status_t st = codec_music_end(dev);
	if (st != CODEC_OK) return st;

	return codec_stop(dev);
}


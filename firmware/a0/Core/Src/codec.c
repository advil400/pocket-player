/*
 * codec.c
 *
 *  Created on: Nov 21, 2025
 *      Author: a400
 */

#include "codec.h"

/* Helpers ^_^*/

static uint16_t codec_hal_adr(const codec_t *dev){
	return (uint16_t)(dev->i2c_adr << 1);
}

static void codec_build_map(uint32_t reg, uint8_t map[3]){
	map[0] = (uint8_t)((reg >> 16) & 0xFFu); /* high [23:16] */
	map[1] = (uint8_t)((reg >> 8)  & 0xFFu); /* mid [15:8] */
	map[2] = (uint8_t)(reg		   & 0xFFu); /* low [7:0] */
}

/* Public API */

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

codec_status_t codec_reg_update_bits(codec_t *dev,
									 uint32_t reg,
									 uint8_t mask,
									 uint8_t value)
{
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








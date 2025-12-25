/*
 * codec.h
 *
 *	Naive Implementation of a I2C Control Plane Configuration for the Cirrus Logic CS43131 audio codec.
 *	Most of the time I'm happy go lucky.
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


/* ADR pin tied to GND, 0x30 (default) */
#define CODEC_I2C_ADR	(0x30u)

/* I2C timeout for control-port operations (ms) */
#define CODEC_I2C_TIMEOUT	(10u)

/* I2C wait after Reset: tPUD >= 1.5ms + margin (ms) */
#define CODEC_TPUD_DELAY 	(4u)

/* RESET low pulse (ms)*/
#define CODEC_RESET_PULSE	(1u)

/*Status Codes*/
typedef enum {
	CODEC_OK	= 0,
	CODEC_PARAM_ERROR = -1,
	CODEC_I2C_ERROR = -2,
	CODEC_ID_MISMATCH = -3
} codec_status_t;

/*Device Context*/
typedef struct {
	I2C_HandleTypeDef *hi2c;
	GPIO_TypeDef *reset_port;
	uint16_t reset_pin;
	uint8_t i2c_adr;
} codec_t;

/* CS43131 Device ID register locations and expected value*/
#define CS43131_REG_DEVID_AB (0x10000u)
#define CS43131_REG_DEVID_CD (0x10001u)
#define CS43131_REG_DEVID_E (0x10002u)

#define CS43131_DEVID_AB (0x43u) /* 0100 0011 */
#define CS43131_DEVID_CD (0x13u) /* 0001 0011 */
#define CS43131_DEVID_E  (0x10u) /* 0001 0000 */

/*Control Plane*/
codec_status_t codec_init(codec_t *dev,
						  I2C_HandleTypeDef *hi2c,
						  GPIO_TypeDef *reset_port,
						  uint16_t reset_pin);

/*Device ID check (no RESET)*/
codec_status_t codec_id(codec_t *dev);

/*8-bit single-register access helpers*/

codec_status_t codec_reg_read8(codec_t *dev,
								uint32_t reg,
								uint8_t *value);

codec_status_t codec_reg_write8(codec_t *dev,
								uint32_t reg,
								uint8_t value);

/*Reserved Bits safety helper*/
codec_status_t codec_reg_update_bits(codec_t *dev,
									 uint32_t reg,
									 uint8_t mask,
									 uint8_t value);

#ifdef __cplusplus
}
#endif


#endif /* CODEC_H_ */

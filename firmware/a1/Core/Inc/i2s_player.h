/*
 * i2s_player.h
 *
 *  Created on: Dec 20, 2025
 *      Author: a400
 */

#pragma once

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "audio_stream.h"

typedef enum {
	PLAYER_STOP = 0,
	PLAYER_RUN,
	PLAYER_ERR
} player_state_t;

typedef struct {

	I2S_HandleTypeDef *hi2s;
	audio_stream_t *stream;

	uint16_t *dma_buf;
	uint32_t dma_words;		// words in buffer (even number)
	volatile player_state_t state;

	volatile uint32_t cb_half;
	volatile uint32_t cb_full;
	volatile uint32_t last_error;
} i2s_player_t;

void i2s_player_init(i2s_player_t *p,
					 I2S_HandleTypeDef *hi2s,
					 audio_stream_t *stream,
					 uint16_t *dma_buf,
					 uint32_t dma_words);

HAL_StatusTypeDef i2s_player_start(i2s_player_t *p);
HAL_StatusTypeDef i2s_player_stop(i2s_player_t *p);

void i2s_player_half(i2s_player_t *p);
void i2s_player_full(i2s_player_t *p);
void i2s_player_error(i2s_player_t *p);



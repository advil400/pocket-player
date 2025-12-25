/*
 * i2s_player.c
 *
 *  Created on: Dec 20, 2025
 *      Author: a400
 */

#include "i2s_player.h"
#include <string.h>

static void fill_half(i2s_player_t *p, uint32_t half_index)
{
	// half_index: 0 = first half, 1 = second half
	const uint32_t half_words = p->dma_words / 2U;
	uint16_t *dst = &p->dma_buf[half_index * half_words];

	// bytes to fill = half_words * 2
	const uint32_t bytes = half_words * 2U;

	// pull PCM bytes (interleaved L/R 16-bit) into the DMA buffer
	(void)stream_read(p->stream, (uint8_t*)dst, bytes);
}

void i2s_player_init(i2s_player_t *p,
					 I2S_HandleTypeDef *hi2s,
					 audio_stream_t *stream,
					 uint16_t *dma_buf,
					 uint32_t dma_words)
{
	memset(p, 0, sizeof(*p));
	p->hi2s = hi2s;
	p->stream = stream;
	p->dma_buf = dma_buf;
	p->dma_words = dma_words;
	p->state = PLAYER_STOP;
}

HAL_StatusTypeDef i2s_player_start(i2s_player_t *p)
{
	if ((p->dma_words == 0U) || ((p->dma_words & 1U) != 0U)) {
		p->state = PLAYER_ERR;
		return HAL_ERROR;
	}

	//Pre-fill
	fill_half(p, 0);
	fill_half(p, 1);

	p->cb_half = 0;
	p->cb_full = 0;
	p->last_error = 0;
	p->state = PLAYER_RUN;

	return HAL_I2S_Transmit_DMA(p->hi2s, p->dma_buf, (uint16_t)p->dma_words);
}

HAL_StatusTypeDef i2s_player_stop(i2s_player_t *p)
{
	p->state = PLAYER_STOP;
	return HAL_I2S_DMAStop(p->hi2s);
}

void i2s_player_half(i2s_player_t *p)
{
	if (p->state != PLAYER_RUN) return;
	p->cb_half++;
	fill_half(p, 0);
}

void i2s_player_full(i2s_player_t *p)
{
	if (p->state != PLAYER_RUN) return;
	p->cb_full++;
	fill_half(p, 1);
}

void i2s_player_error(i2s_player_t *p)
{
	p->last_error = p->hi2s->ErrorCode;
	p->state = PLAYER_ERR;
}

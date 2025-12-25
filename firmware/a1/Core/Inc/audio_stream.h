/*
 * audio_stream.h
 *
 *  Created on: Dec 20, 2025
 *      Author: a400
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "ff.h"

typedef enum {
	AUDIO_OK = 0,
	AUDIO_ERR_FATFS,
	AUDIO_ERR_WAV_FORMAT,
	AUDIO_ERR_EOF,
} audio_status_t;

typedef struct {
	uint16_t audio_format;		// 1 = PCM
	uint16_t num_chan;			// 1 or 2 (Mono or Stereo channels)
	uint32_t sample_rate;		// 44.1kHz or other
	uint32_t byte_rate;
	uint16_t block_align;
	uint16_t bit_depth;			// 16bit or other
	uint32_t data_bytes;		// PCM data size
	uint32_t data_start;		// file offset to PCM data
} wav_info_t;

typedef struct {
	FIL file;
	wav_info_t wav;

	// Ring-buffer for read-ahead PCM bytes (power-of-2 size)
	uint8_t *rb;
	uint32_t rb_size;
	uint32_t rb_mask;
	volatile uint32_t w;		// write index (bytes)
	volatile uint32_t r;		// read index (bytes)

	uint32_t rem;				// Remaining PCM bytes in file (for end-of-stream)

	// Debug counters
	volatile uint32_t uf;		// underflows
	FRESULT last_fr;
} audio_stream_t;

/* Init audio stream with external ring buffer memory (power-of-2 size and 4-byte aligned) */

void stream_init(audio_stream_t *s, uint8_t *rb_mem, uint32_t rb_size);

/* Mount SD volume */

audio_status_t stream_mount(const TCHAR *drive);

/* Unmount SD volume */

audio_status_t stream_unmount(const TCHAR *drive);

/* Open and parse WAV, seek to PCM data, reset ring buffer */

audio_status_t parse_wav(audio_stream_t *s, const TCHAR *path);

/* Close file (does not unmount) */

void stream_close(audio_stream_t *s);

/* Push read-ahead: read more PCM from file into ring buffer. Main loop not IRQ. */

audio_status_t stream_push(audio_stream_t *s);

/* Bytes available to consume from ring buffer */

uint32_t stream_avail(const audio_stream_t *s);

/* Consume up to 'len' bytes from ring buffer into dst.
 * If not enough data, fill remainder with 0 and increment underflow counter.
 * Safe to call from I2S DMA callback later (single consumer).
 */

uint32_t stream_read(audio_stream_t *s, void *dst, uint32_t len);

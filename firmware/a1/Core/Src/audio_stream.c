/*
 * audio_stream.c
 *
 *  Created on: Dec 20, 2025
 *      Author: a400
 */


#include "audio_stream.h"
#include "fatfs.h"
#include <string.h>

#ifndef AUDIO_READ_CHUNK
#define AUDIO_READ_CHUNK (16U * 1024U)		// file read granularity
#endif

/* Little-endian helpers */
static uint16_t rd16(const uint8_t *p) {
	return(uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t rd32(const uint8_t *p) {
	return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

static int tag_eq(const uint8_t *p, const char t[4]) {
	return (p[0]==(uint8_t)t[0] && p[1]==(uint8_t)t[1] && p[2]==(uint8_t)t[2] && p[3]==(uint8_t)t[3]);
}

static FRESULT read_exact(FIL *f, void *dst, UINT n)
{
	UINT br = 0;
	FRESULT fr = f_read(f, dst, n, &br);
	if (fr != FR_OK) return fr;
	return (br == n) ? FR_OK : FR_INT_ERR;	// treat short read here as error during header parse
}

/* Ring buffer (single producer, single consumer) */
void stream_init(audio_stream_t *s, uint8_t *rb_mem, uint32_t rb_size)
{
	memset(s, 0, sizeof(*s));
	s->rb = rb_mem;
	s->rb_size = rb_size;
	s->rb_mask = rb_size - 1U;
	s->w = 0;
	s->r = 0;
	s->rem = 0;
	s->uf = 0;
	s->last_fr = FR_OK;
}

static uint32_t rb_used(const audio_stream_t *s) {
	return (s->w - s->r) & s->rb_mask;
}

static uint32_t rb_free(const audio_stream_t *s) {
	return (s->rb_size - 1U) - rb_used(s);	// keep one byte empty to explicit full vs empty
}

uint32_t stream_avail(const audio_stream_t *s) {
	return rb_used(s);
}

static uint32_t rb_write(const audio_stream_t *s, uint8_t **ptr)
{
	uint32_t wi = s->w & s->rb_mask;
	uint32_t end = s->rb_size - wi;
	uint32_t free = rb_free(s);
	uint32_t contig = (free < end) ? free : end;
	*ptr = &s->rb[wi];
	return contig;
}

static void rb_push_write(audio_stream_t *s, uint32_t n)
{
	s->w = (s->w + n) & s->rb_mask;
}

static uint32_t rb_read(const audio_stream_t *s, uint8_t **ptr)
{
	uint32_t ri = s->r & s->rb_mask;
	uint32_t end = s->rb_size - ri;
	uint32_t used = rb_used(s);
	uint32_t contig = (used < end) ? used : end;
	*ptr = &s->rb[ri];
	return contig;
}

static void rb_push_read(audio_stream_t *s, uint32_t n)
{
	s->r = (s->r + n) & s->rb_mask;
}

/* SD mount helpers */

audio_status_t stream_mount(const TCHAR *drive)
{
	FRESULT fr = f_mount(&SDFatFS, drive, 1);
	return (fr == FR_OK) ? AUDIO_OK : AUDIO_ERR_FATFS;
}

audio_status_t stream_unmount(const TCHAR *drive)
{
	FRESULT fr = f_mount(NULL, drive, 1);
	return (fr == FR_OK) ? AUDIO_OK : AUDIO_ERR_FATFS;
}

/* WAV Parsing */

static audio_status_t parse_wav_header(FIL *f, wav_info_t *w, FRESULT *out_fr)
{
	uint8_t hdr12[12];
	uint8_t chdr[8];

	memset(w, 0, sizeof(*w));

	FRESULT fr = f_lseek(f, 0);
	if (fr != FR_OK) {
		if (out_fr) *out_fr = fr;
		return AUDIO_ERR_FATFS;
	}

	fr = read_exact(f, hdr12, sizeof(hdr12));
	if (fr != FR_OK) {
		if (out_fr) *out_fr = fr;
		return AUDIO_ERR_FATFS;
	}

	if (!tag_eq(hdr12 + 0, "RIFF") || !tag_eq(hdr12 + 8, "WAVE")) {
		if (out_fr) *out_fr = FR_OK;
		return AUDIO_ERR_WAV_FORMAT;
	}

	int got_fmt = 0;
	int got_data = 0;

	while (!got_data) {
		UINT br = 0;
		fr = f_read(f, chdr, sizeof(chdr), &br);
		if (fr != FR_OK){
			if (out_fr) *out_fr = fr;
			return AUDIO_ERR_FATFS;
		}
		if (br != sizeof(chdr)) {
			if (out_fr) *out_fr = FR_OK;
			return AUDIO_ERR_WAV_FORMAT;
		}

		uint32_t chunk_size = rd32(chdr + 4);

		if (tag_eq(chdr + 0, "fmt ")) {
			uint8_t fmt16[16];							// read the first 16 bytes of fmt (pcm minimum)
			if (chunk_size < 16) {
				if (out_fr) *out_fr = FR_OK;
				return AUDIO_ERR_WAV_FORMAT;
			}
			fr = read_exact(f, fmt16, sizeof(fmt16));
			if (fr != FR_OK) {
				if (out_fr) *out_fr = fr;
				return AUDIO_ERR_FATFS;
			}

			w->audio_format  	= rd16(fmt16 + 0);
			w->num_chan 		= rd16(fmt16 + 2);
			w->sample_rate 		= rd32(fmt16 + 4);
			w->byte_rate		= rd32(fmt16 + 8);
			w->block_align		= rd16(fmt16 + 12);
			w->bit_depth		= rd16(fmt16 + 14);

			uint32_t remain = chunk_size - 16;			// skip any remaining fmt bytes
			if (remain) {
				fr = f_lseek(f, f_tell(f) + remain);
				if (fr != FR_OK) {
					if (out_fr) *out_fr = fr;
					return AUDIO_ERR_FATFS;
				}
			}
			got_fmt = 1;
		}
		else if (tag_eq(chdr + 0, "data")){
			w->data_bytes = chunk_size;
			w->data_start = (uint32_t)f_tell(f);
			got_data = 1;
		}
		else {
			fr = f_lseek(f, f_tell(f) + chunk_size);	// skip unknown check
			if (fr != FR_OK) {
				if (out_fr) *out_fr = fr;
				return AUDIO_ERR_FATFS;
			}
			if (got_data && !got_fmt){
				if (out_fr) *out_fr = FR_OK;
				return AUDIO_ERR_WAV_FORMAT;
			}
		}

		if (chunk_size & 1U) {							// chunks are word-aligned: skip pad byte if chunk_size is odd
			fr = f_lseek(f, f_tell(f) + 1);
			if (fr != FR_OK) {
				if(out_fr) *out_fr = fr;
				return AUDIO_ERR_FATFS;
			}
		}

		if (got_data && !got_fmt) {						// if we found "data" before "fmt, still require fmt eventually (strict for now)
			if (out_fr) *out_fr = FR_OK;
			return AUDIO_ERR_WAV_FORMAT;
		}
	}

	if (w->audio_format !=1) {
		if (out_fr) *out_fr = FR_OK;
		return AUDIO_ERR_WAV_FORMAT;
	}

	if (out_fr) *out_fr = FR_OK;
	return AUDIO_OK;
}

audio_status_t parse_wav(audio_stream_t *s, const TCHAR *path)
{
	s->w = 0;
	s->r = 0;
	s->rem = 0;
	s->uf = 0;

	FRESULT fr = f_open(&s->file, path, FA_READ);
	s->last_fr = fr;
	if (fr != FR_OK) return AUDIO_ERR_FATFS;

	audio_status_t st = parse_wav_header(&s->file, &s->wav, &s->last_fr);
	if (st != AUDIO_OK) {
		f_close(&s->file);
		return st;
	}

	fr = f_lseek(&s->file, s->wav.data_start);		// seek to PCM data
	s->last_fr = fr;
	if (fr != FR_OK) {
		f_close(&s->file);
		return AUDIO_ERR_FATFS;
	}

	s->rem = s->wav.data_bytes;
	return AUDIO_OK;
}

void stream_close(audio_stream_t *s)
{
	(void)f_close(&s->file);
}

audio_status_t stream_push(audio_stream_t *s)
{
	while (s->rem > 0) {							// fill until either ring buffer full or end of file
		uint32_t free = rb_free(s);
		if (free <1024U) break;

		uint32_t want = free;
		if (want > AUDIO_READ_CHUNK) want = AUDIO_READ_CHUNK;
		if (want > s->rem) want = s->rem;

		uint8_t *wptr;
		uint32_t contig = rb_write(s, &wptr);
		if (contig == 0) break;

		uint32_t part = (want < contig) ? want : contig;
		UINT br = 0;
		FRESULT fr = f_read(&s->file, wptr, (UINT)part, &br);
		s->last_fr = fr;
		if (fr != FR_OK) return AUDIO_ERR_FATFS;
		if (br == 0) return AUDIO_ERR_EOF;

		rb_push_write(s, br);
		s->rem -= br;

		if (br < part){
			return AUDIO_ERR_EOF;
		}

		want -= br;
		if (want == 0 || s->rem == 0) continue;

		contig = rb_write(s, &wptr);
		if (contig == 0) break;

		part = (want < contig) ? want : contig;
		br = 0;
		fr = f_read(&s->file, wptr, (UINT)part, &br);
		s->last_fr = fr;
		if (fr != FR_OK) return AUDIO_ERR_FATFS;
		if (br == 0) return AUDIO_ERR_EOF;

		rb_push_write(s, br);
		s->rem -= br;

		if (br < part) return AUDIO_ERR_EOF;
	}

	return AUDIO_OK;
}

uint32_t stream_read(audio_stream_t *s, void *dst, uint32_t len)
{
	uint8_t *out = (uint8_t *)dst;
	uint32_t done = 0;

	while (done < len) {
		uint8_t *rptr;
		uint32_t contig = rb_read(s, &rptr);
		if (contig == 0) {
			memset(out + done, 0, len - done);
			s->uf++;
			return len;
		}

		uint32_t n = (len - done < contig) ? (len - done) : contig;
		memcpy(out + done, rptr, n);
		rb_push_read(s, n);
		done += n;
	}

	return done;
}


/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2024 libsigrok contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Driver for the Hantek DSO2xxx series oscilloscopes using the
 * "quick-fetch" firmware patch by phmarek
 * (https://github.com/phmarek/hantek-dso2000-quick-fetch).
 *
 * The patched firmware exposes a TCP server (default port 5025) on the
 * scope's network interface (requires DavidAlfa's USB-networking kernel).
 * When the user presses SAVE TO USB the scope sends one binary frame:
 *
 *   [frame_header]  (struct hantek_frame_header, 32 bytes, little-endian)
 *   [ch1_samples]   (num_samples × int8_t, only if ch1_enabled != 0)
 *   [ch2_samples]   (num_samples × int8_t, only if ch2_enabled != 0)
 *
 * The header layout has been inferred from the Perl receiver script:
 *
 *   uint32_t  magic;          // 0x48544B32 = "HTK2"
 *   uint32_t  num_samples;    // number of samples per channel
 *   float     time_per_div;   // seconds per division
 *   float     sample_period;  // seconds between samples  (= 1/samplerate)
 *   uint8_t   ch1_enabled;
 *   uint8_t   ch2_enabled;
 *   uint8_t   _reserved[2];
 *   float     ch1_scale;      // volts per division
 *   float     ch2_scale;
 *   int32_t   ch1_offset_raw; // raw offset (signed ADC units)
 *   int32_t   ch2_offset_raw;
 *
 * Each sample is a signed 8-bit integer in raw ADC units.
 * Conversion:  voltage = (raw_sample - offset_raw) * (scale / 25.0f)
 * (25 ADC counts per division, 8 divisions visible → 200 count full range)
 */

#ifndef LIBSIGROK_HARDWARE_HANTEK_DSO2XXX_PROTOCOL_H
#define LIBSIGROK_HARDWARE_HANTEK_DSO2XXX_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "hantek-dso2xxx"

/** Magic bytes at the start of every frame: ASCII "HTK2". */
#define HANTEK_FRAME_MAGIC     0x32KTH  /* little-endian: 'H','T','K','2' */
#define HANTEK_FRAME_MAGIC_LE  0x32KTH

/*
 * Use the actual 4-byte value directly so the compiler doesn't trip over
 * the macro above – keep it readable via a comment.
 */
#undef  HANTEK_FRAME_MAGIC_LE
/* 'H'=0x48 'T'=0x54 'K'=0x4B '2'=0x32  → little-endian uint32 = 0x324B5448 */
#define HANTEK_FRAME_MAGIC_LE  UINT32_C(0x324B5448)

/** Default TCP port exposed by the patched firmware. */
#define HANTEK_DEFAULT_PORT    "5025"

/** Default TCP address (scope must be on the network via USB-networking). */
#define HANTEK_DEFAULT_ADDR    "192.168.7.1"

/** I/O timeout for TCP operations, in milliseconds. */
#define HANTEK_TCP_TIMEOUT_MS  60000

/** Number of ADC counts per oscilloscope division. */
#define HANTEK_COUNTS_PER_DIV  25

#pragma pack(push, 1)
/**
 * Binary frame header sent by the patched DSO firmware.
 * All multi-byte fields are little-endian (the scope runs on ARM/Allwinner).
 */
struct hantek_frame_header {
	uint32_t magic;           /**< Must equal HANTEK_FRAME_MAGIC_LE. */
	uint32_t num_samples;     /**< Samples per enabled channel. */
	float    time_per_div;    /**< Seconds per timebase division. */
	float    sample_period;   /**< Seconds between consecutive samples. */
	uint8_t  ch1_enabled;     /**< Non-zero if CH1 data follows. */
	uint8_t  ch2_enabled;     /**< Non-zero if CH2 data follows. */
	uint8_t  reserved[2];
	float    ch1_scale;       /**< CH1 volts per division. */
	float    ch2_scale;       /**< CH2 volts per division. */
	int32_t  ch1_offset_raw;  /**< CH1 vertical offset in ADC counts. */
	int32_t  ch2_offset_raw;  /**< CH2 vertical offset in ADC counts. */
};
#pragma pack(pop)

/** Per-device instance state. */
struct dev_context {
	/* Network connection */
	char    *tcp_address;     /**< IP / hostname of the scope. */
	char    *tcp_port;        /**< TCP port string. */
	int      sockfd;          /**< Connected socket fd, or -1. */

	/* Acquisition state */
	gboolean acq_running;
	uint64_t frames_received;

	/* Channel metadata filled in from the most recent header */
	uint32_t num_samples;
	float    sample_period;   /**< 1 / samplerate */
	uint8_t  ch1_enabled;
	uint8_t  ch2_enabled;
	float    ch1_scale;
	float    ch2_scale;
	int32_t  ch1_offset_raw;
	int32_t  ch2_offset_raw;
};

/* Forward declarations for api.c ↔ protocol.c interface */
SR_PRIV int  hantek_dso2xxx_tcp_connect(const struct sr_dev_inst *sdi);
SR_PRIV void hantek_dso2xxx_tcp_close(const struct sr_dev_inst *sdi);
SR_PRIV int  hantek_dso2xxx_receive_frame(const struct sr_dev_inst *sdi);

#endif /* LIBSIGROK_HARDWARE_HANTEK_DSO2XXX_PROTOCOL_H */

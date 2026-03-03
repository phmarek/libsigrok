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
 */

#ifndef LIBSIGROK_HARDWARE_HANTEK_DSO2XXX_PROTOCOL_H
#define LIBSIGROK_HARDWARE_HANTEK_DSO2XXX_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "hantek-dso2xxx"

/**
 * Hantek DSO2xxx packet header.
 * All fields are ASCII-encoded decimal strings (NOT null-terminated).
 * Total size: 128 bytes.
 */
typedef struct __attribute__((packed)) {
    char magic[2];            /* data[0-1]:      Packet identifier, always "#9" */
    char packet_length[9];    /* data[2-10]:     Byte length of current packet */
    char total_length[9];     /* data[11-19]:    Total byte length of all data */
    char uploaded_length[9];  /* data[20-28]:    Byte length of uploaded data */
    char run_status;          /* data[29]:       Current running status */
    char trigger_status;      /* data[30]:       Trigger status */
    char ch1_offset[4];       /* data[31-34]:    Channel 1 vertical offset */
    char ch2_offset[4];       /* data[35-38]:    Channel 2 vertical offset */
    char ch3_offset[4];       /* data[39-42]:    Channel 3 vertical offset */
    char ch4_offset[4];       /* data[43-46]:    Channel 4 vertical offset */
    char ch1_voltage[7];      /* data[47-53]:    Channel 1 voltage scale */
    char ch2_voltage[7];      /* data[54-60]:    Channel 2 voltage scale */
    char ch3_voltage[7];      /* data[61-67]:    Channel 3 voltage scale */
    char ch4_voltage[7];      /* data[68-74]:    Channel 4 voltage scale */
    char ch_enable[4];        /* data[75-78]:    Channel enable flags (CH1-CH4, one char each) */
    char sample_rate[9];      /* data[79-87]:    Sampling rate */
    char sample_multiple[6];  /* data[88-93]:    Sampling multiple */
    char trigger_time[9];     /* data[94-102]:   Display trigger time of current frame */
    char acq_start_time[9];   /* data[103-111]:  Acquisition start time point of current frame */
    char reserved[16];        /* data[112-127]:  Reserved */
} hantek_packet_header_t;

_Static_assert(sizeof(hantek_packet_header_t) == 128,
               "hantek_packet_header_t must be exactly 128 bytes");


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

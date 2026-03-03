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

/**
 * @file protocol.c
 *
 * Low-level TCP communication and binary frame decoding for the
 * Hantek DSO2xxx quick-fetch driver.
 *
 * Protocol overview
 * -----------------
 * The patched scope firmware (LD_PRELOAD'd quick-fetch.so) listens on a
 * TCP port.  When the user presses SAVE TO USB the scope transmits a single
 * binary blob per trigger event:
 *
 *   1. struct hantek_frame_header  (32 bytes, all fields little-endian)
 *   2. CH1 sample data             (num_samples × int8_t, if ch1_enabled)
 *   3. CH2 sample data             (num_samples × int8_t, if ch2_enabled)
 *
 * The connection stays open between frames; we simply block-read the next
 * header to detect the next waveform.
 */

#include <config.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

/**
 * Perform a full blocking read of exactly @p len bytes into @p buf.
 *
 * Returns SR_OK on success, SR_ERR_IO on any error or EOF.
 */
static int tcp_read_all(int fd, void *buf, size_t len)
{
	uint8_t *ptr = (uint8_t *)buf;
	size_t   remaining = len;
	ssize_t  n;

	while (remaining > 0) {
		n = read(fd, ptr, remaining);
		if (n <= 0) {
			if (n < 0 && (errno == EINTR || errno == EAGAIN))
				continue;
			sr_err("TCP read error: %s (wanted %zu, got %zd)",
			       n < 0 ? strerror(errno) : "EOF",
			       len, (ssize_t)(len - remaining));
			return SR_ERR_IO;
		}
		ptr       += n;
		remaining -= (size_t)n;
	}
	return SR_OK;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/**
 * Open a TCP connection to the scope.
 *
 * Populates devc->sockfd on success.
 */
SR_PRIV int hantek_dso2xxx_tcp_connect(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct addrinfo     hints, *res, *rp;
	int                 fd, rc, one = 1;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family   = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	rc = getaddrinfo(devc->tcp_address, devc->tcp_port, &hints, &res);
	if (rc != 0) {
		sr_err("getaddrinfo(%s:%s): %s",
		       devc->tcp_address, devc->tcp_port, gai_strerror(rc));
		return SR_ERR;
	}

	fd = -1;
	for (rp = res; rp; rp = rp->ai_next) {
		fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (fd < 0)
			continue;
		if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0)
			break;
		close(fd);
		fd = -1;
	}
	freeaddrinfo(res);

	if (fd < 0) {
		sr_err("Could not connect to %s:%s",
		       devc->tcp_address, devc->tcp_port);
		return SR_ERR;
	}

	/* Disable Nagle for lower latency on the control path. */
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

	devc->sockfd = fd;
	sr_info("Connected to Hantek DSO2xxx at %s:%s",
	        devc->tcp_address, devc->tcp_port);
	return SR_OK;
}

/**
 * Close the TCP connection.
 */
SR_PRIV void hantek_dso2xxx_tcp_close(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	if (devc->sockfd >= 0) {
		close(devc->sockfd);
		devc->sockfd = -1;
		sr_dbg("TCP connection closed.");
	}
}

/**
 * Read one complete waveform frame from the scope and dispatch
 * SR_DF_ANALOG packets for each enabled channel.
 *
 * This function blocks until the full frame has been received.
 *
 * @return SR_OK on success, SR_ERR* on failure.
 */
SR_PRIV int hantek_dso2xxx_receive_frame(const struct sr_dev_inst *sdi)
{
	struct dev_context      *devc     = sdi->priv;
	struct hantek_frame_header hdr;
	int8_t                  *raw_buf  = NULL;
	float                   *float_buf = NULL;
	int                      ret      = SR_ERR;
	uint32_t                 ns;
	size_t                   ch_bytes;
	uint32_t                 i;

	/* ------------------------------------------------------------------ *
	 * 1.  Read and validate the frame header.                             *
	 * ------------------------------------------------------------------ */
	if (tcp_read_all(devc->sockfd, &hdr, sizeof(hdr)) != SR_OK) {
		sr_err("Failed to read frame header.");
		return SR_ERR_IO;
	}

	if (GUINT32_FROM_LE(hdr.magic) != HANTEK_FRAME_MAGIC_LE) {
		sr_err("Bad frame magic: 0x%08X (expected 0x%08X)",
		       GUINT32_FROM_LE(hdr.magic), HANTEK_FRAME_MAGIC_LE);
		return SR_ERR_DATA;
	}

	/* Byte-swap all fields from little-endian to host order. */
	ns = GUINT32_FROM_LE(hdr.num_samples);
	if (ns == 0 || ns > 8 * 1024 * 1024) {
		sr_err("Implausible sample count: %u", ns);
		return SR_ERR_DATA;
	}

	/*
	 * The float fields are transmitted in the endianness of the scope's
	 * ARM CPU (little-endian).  On little-endian hosts this is a no-op;
	 * on big-endian hosts we byte-swap via the 32-bit integer trick.
	 */
#if G_BYTE_ORDER == G_BIG_ENDIAN
	{
		uint32_t tmp;
# define BSWAP_FLOAT(f) do { memcpy(&tmp, &(f), 4); \
		tmp = GUINT32_SWAP_LE_BE(tmp);         \
		memcpy(&(f), &tmp, 4); } while (0)
		BSWAP_FLOAT(hdr.sample_period);
		BSWAP_FLOAT(hdr.time_per_div);
		BSWAP_FLOAT(hdr.ch1_scale);
		BSWAP_FLOAT(hdr.ch2_scale);
# undef BSWAP_FLOAT
		hdr.ch1_offset_raw = (int32_t)GUINT32_SWAP_LE_BE(
		                         (uint32_t)hdr.ch1_offset_raw);
		hdr.ch2_offset_raw = (int32_t)GUINT32_SWAP_LE_BE(
		                         (uint32_t)hdr.ch2_offset_raw);
	}
#endif

	/* Cache header metadata in device context for later inspection. */
	devc->num_samples    = ns;
	devc->sample_period  = hdr.sample_period;
	devc->ch1_enabled    = hdr.ch1_enabled;
	devc->ch2_enabled    = hdr.ch2_enabled;
	devc->ch1_scale      = hdr.ch1_scale;
	devc->ch2_scale      = hdr.ch2_scale;
	devc->ch1_offset_raw = hdr.ch1_offset_raw;
	devc->ch2_offset_raw = hdr.ch2_offset_raw;

	sr_dbg("Frame: %u samples, period=%.3e s, CH1=%s CH2=%s",
	       ns, hdr.sample_period,
	       hdr.ch1_enabled ? "on" : "off",
	       hdr.ch2_enabled ? "on" : "off");

	/* ------------------------------------------------------------------ *
	 * 2.  Allocate receive and conversion buffers.                        *
	 * ------------------------------------------------------------------ */
	ch_bytes = (size_t)ns * sizeof(int8_t);
	raw_buf  = g_malloc(ch_bytes);
	if (!raw_buf) {
		sr_err("Out of memory allocating raw sample buffer (%zu bytes).",
		       ch_bytes);
		return SR_ERR_MALLOC;
	}

	float_buf = g_malloc(ns * sizeof(float));
	if (!float_buf) {
		sr_err("Out of memory allocating float sample buffer.");
		g_free(raw_buf);
		return SR_ERR_MALLOC;
	}

	/* ------------------------------------------------------------------ *
	 * 3.  Send SR_DF_FRAME_BEGIN                                          *
	 * ------------------------------------------------------------------ */
	std_session_send_df_frame_begin(sdi);

	/* ------------------------------------------------------------------ *
	 * 4.  Process each enabled channel.                                   *
	 * ------------------------------------------------------------------ */

	/*
	 * Helper lambda-like macro to read and dispatch one channel.
	 *
	 * @param CH_IDX    0-based channel index into sdi->channels
	 * @param ENABLED   hdr.ch1_enabled / hdr.ch2_enabled
	 * @param SCALE     hdr.ch1_scale / hdr.ch2_scale
	 * @param OFFSET    hdr.ch1_offset_raw / hdr.ch2_offset_raw
	 * @param LABEL     string literal for log messages
	 */
#define DISPATCH_CHANNEL(CH_IDX, ENABLED, SCALE, OFFSET, LABEL)         \
	do {                                                                 \
		struct sr_channel   *ch;                                     \
		struct sr_datafeed_packet  pkt;                              \
		struct sr_datafeed_analog  analog;                           \
		struct sr_analog_encoding  enc;                              \
		struct sr_analog_meaning   meaning;                          \
		struct sr_analog_spec      spec;                             \
		GSList *chl = NULL;                                          \
		                                                             \
		if (!(ENABLED))                                              \
			break;                                               \
		                                                             \
		/* Read raw signed 8-bit samples from the stream. */         \
		if (tcp_read_all(devc->sockfd, raw_buf, ch_bytes) != SR_OK) { \
			sr_err("Failed to read " LABEL " samples.");         \
			ret = SR_ERR_IO;                                     \
			goto cleanup;                                        \
		}                                                            \
		                                                             \
		/*                                                           \
		 * Convert raw ADC counts to volts:                          \
		 *   voltage = (raw - offset_raw) * (scale / counts_per_div) \
		 */                                                          \
		for (i = 0; i < ns; i++) {                                   \
			float_buf[i] =                                       \
			    ((float)raw_buf[i] - (float)(OFFSET)) *          \
			    ((SCALE) / (float)HANTEK_COUNTS_PER_DIV);        \
		}                                                            \
		                                                             \
		/* Locate the sr_channel object for this channel index. */   \
		ch = g_slist_nth_data(sdi->channels, (CH_IDX));             \
		if (!ch || !ch->enabled) {                                   \
			sr_dbg(LABEL " not enabled in session, skipping.");  \
			break;                                               \
		}                                                            \
		chl = g_slist_append(NULL, ch);                              \
		                                                             \
		sr_analog_init(&analog, &enc, &meaning, &spec, 6);          \
		meaning.mq      = SR_MQ_VOLTAGE;                             \
		meaning.unit    = SR_UNIT_VOLT;                              \
		meaning.mqflags = SR_MQFLAG_DC;                              \
		meaning.channels = chl;                                      \
		analog.data      = float_buf;                                \
		analog.num_samples = ns;                                     \
		                                                             \
		pkt.type    = SR_DF_ANALOG;                                  \
		pkt.payload = &analog;                                       \
		sr_session_send(sdi, &pkt);                                  \
		g_slist_free(chl);                                           \
	} while (0)

	DISPATCH_CHANNEL(0, hdr.ch1_enabled, hdr.ch1_scale,
	                 hdr.ch1_offset_raw, "CH1");
	DISPATCH_CHANNEL(1, hdr.ch2_enabled, hdr.ch2_scale,
	                 hdr.ch2_offset_raw, "CH2");

#undef DISPATCH_CHANNEL

	/* ------------------------------------------------------------------ *
	 * 5.  Send SR_DF_FRAME_END                                            *
	 * ------------------------------------------------------------------ */
	std_session_send_df_frame_end(sdi);

	devc->frames_received++;
	ret = SR_OK;

cleanup:
	g_free(float_buf);
	g_free(raw_buf);
	return ret;
}

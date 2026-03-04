# hantek-dso2xxx libsigrok driver

A libsigrok driver for the Hantek DSO2xxx oscilloscope series (DSO2C10,
DSO2C15, DSO2D10, DSO2D15) that receives waveform data over TCP using the
quick-fetch firmware patch by phmarek:
https://github.com/phmarek/hantek-dso2000-quick-fetch

Initial contents generated via Claude.AI, https://claude.ai/chat/b934e60d-d427-4455-a683-f8793332eb8a.

---

## How it works

The patched firmware (quick-fetch.so, `LD_PRELOAD`'d on the scope) listens
on a TCP port. When the user presses `SAVE_TO_USB` the scope freezes the
current waveform and streams a compact binary frame to any connected client.

Binary frame layout (all fields little-endian):

    Offset  Size  Type     Field
    ------  ----  -------  --------------------------------
    0       4     uint32   magic = 0x324B5448  ("HTK2")
    4       4     uint32   num_samples (per enabled channel)
    8       4     float32  time_per_div   (seconds/division)
    12      4     float32  sample_period  (seconds; 1/samplerate)
    16      1     uint8    ch1_enabled  (non-zero if CH1 data follows)
    17      1     uint8    ch2_enabled  (non-zero if CH2 data follows)
    18      2     uint8[2] reserved
    20      4     float32  ch1_scale  (volts/division)
    24      4     float32  ch2_scale
    28      4     int32    ch1_offset_raw  (signed ADC counts)
    32      4     int32    ch2_offset_raw

After the header: CH1 samples (num_samples x int8, if ch1_enabled != 0),
then CH2 samples (num_samples x int8, if ch2_enabled != 0).

Voltage conversion:
    voltage = (raw_sample - offset_raw) * (scale_V_per_div / 25.0)
(25 ADC counts per division; 8 divisions gives ~200 count full range)

---

## Prerequisites

1. Install the quick-fetch patch on your scope following the instructions
   at https://github.com/phmarek/hantek-dso2000-quick-fetch

2. Use DavidAlfa's USB-networking kernel so the scope appears at
   192.168.7.1 via USB networking. Without this, the TCP server is not
   reachable.

---

## Building

Place the three source files in src/hardware/hantek-dso2xxx/ inside the
libsigrok source tree, then:

1. Add the Makefile.am.fragment content to src/hardware/Makefile.am.
2. Add HW_HANTEK_DSO2XXX enable/disable boilerplate to configure.ac
   (copy the pattern from any comparable driver such as rigol-ds).
3. Re-run ./autogen.sh && ./configure && make

---

## Usage

    # Single capture (waits for one SAVE TO USB press)
    sigrok-cli -d hantek-dso2xxx:conn=192.168.7.1/5025 -o capture.sr

    # Continuous (keeps waiting for further presses)
    sigrok-cli -d hantek-dso2xxx:conn=192.168.7.1/5025 --continuous

    # Custom address/port
    sigrok-cli -d hantek-dso2xxx:conn=10.0.0.5/5025 -o capture.sr

Open the .sr file in PulseView for graphical display.

---

## No configurable acquisition options

All scope settings (timebase, volts/div, coupling, trigger, memory depth)
are configured directly on the instrument front panel. The driver reads
them passively from the binary frame header.

Session-level keys:

    conn           GET   Returns "address/port" string
    samplerate     GET   Derived from sample_period field
    limit_frames   GET   Count of frames received so far

---

## Firmware versions

The quick-fetch patch supports:
  3.0.0(220727.00) - 2022-07-27
  3.0.0(230327.00) - 2023-03-27
  3.0.1(250418.00) - 2025-04-18 (patch v1.3+)

---

## License

GPL-3.0-or-later, matching the rest of libsigrok.

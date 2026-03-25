# libpsk — PSK31/63/125 Modem Library

A zero-dependency, C99 PSK31/63/125 modem library under the MIT license. Suitable
for embedding in Linux applications, microcontrollers (ESP32-S3), or any environment
that can provide float32 audio buffers.

---

## Supported modes

| Mode | Baud | BW | FEC | Bits/symbol |
|------|------|----|-----|-------------|
| BPSK31 | 31.25 | ~31 Hz | None | 1 |
| QPSK31 | 31.25 | ~62 Hz | K=5 conv. | 2 |
| BPSK63 | 62.5 | ~63 Hz | None | 1 |
| BPSK125 | 125.0 | ~125 Hz | None | 1 |
| QPSK63 | 62.5 | ~126 Hz | K=5 conv. | 2 |
| QPSK125 | 125.0 | ~250 Hz | K=5 conv. | 2 |

Audio format: **float32 PCM at 8000 Hz** (mono). All modes work at a fixed carrier
frequency that you choose at context creation (typically 1000–2000 Hz within the
audio passband).

---

## Build

### Linux / macOS / Raspberry Pi

```bash
cmake -B build -S .
cmake --build build
ctest --test-dir build   # run all five tests
```

### Cross-compile for Raspberry Pi (aarch64)

```bash
cmake -B build-rpi -S . \
      -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-raspberrypi-aarch64.cmake
cmake --build build-rpi
```

### Cross-compile for Windows (mingw64)

```bash
cmake -B build-win -S . \
      -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-windows-mingw64.cmake
cmake --build build-win
```

### ESP32-S3 (via ESP-IDF)

See [apps/esp32_modem/README](#esp32-modem) below. The library builds as a standard
ESP-IDF component; no changes to `src/` are required.

---

## Library API

Include `include/psk.h`.  Link with `libpsk` and `-lm`.

### Modes

```c
typedef enum {
    PSK_MODE_BPSK31  = 0,
    PSK_MODE_QPSK31  = 1,
    PSK_MODE_BPSK63  = 2,
    PSK_MODE_BPSK125 = 3,
    PSK_MODE_QPSK63  = 4,
    PSK_MODE_QPSK125 = 5,
} psk_mode_t;
```

### Transmitter

```c
/* Create / destroy */
psk_tx_t *psk_tx_create(unsigned sample_rate, float carrier_hz, psk_mode_t mode);
void      psk_tx_destroy(psk_tx_t *tx);

/* Enqueue text to send */
size_t psk_tx_write(psk_tx_t *tx, const char *text, size_t len);

/* Generate PCM samples — call in a loop to fill your audio output buffer.
 * Outputs ramped silence when nothing is queued. */
size_t psk_tx_read(psk_tx_t *tx, float *out, size_t frames);

/* Non-zero when buffer is empty and the final idle tail has been transmitted */
int psk_tx_idle(psk_tx_t const *tx);
```

### Receiver

```c
/* Callbacks */
typedef void (*psk_rx_char_cb)  (char c,    void *userdata);
typedef void (*psk_rx_status_cb)(float snr_db, float freq_offset_hz, void *userdata);

/* Create / destroy */
psk_rx_t *psk_rx_create(unsigned sample_rate, float carrier_hz, psk_mode_t mode);
void      psk_rx_destroy(psk_rx_t *rx);

/* Register callbacks (both optional) */
void psk_rx_set_char_callback  (psk_rx_t *rx, psk_rx_char_cb   cb, void *userdata);
void psk_rx_set_status_callback(psk_rx_t *rx, psk_rx_status_cb cb, void *userdata);

/* Feed PCM samples — callbacks fire synchronously on this call */
void psk_rx_feed(psk_rx_t *rx, const float *buf, size_t frames);

/* Reset decoder state (e.g., after squelch re-opens) */
void psk_rx_reset(psk_rx_t *rx);
```

The `status_cb` fires approximately once per symbol period. `snr_db` is an
instantaneous estimate; values below −20 dB typically indicate noise only.
`freq_offset_hz` is the Costas loop's measured carrier offset in Hz.

### Varicode utilities

```c
/* Encode one ASCII char to its varicode bit sequence.
 * bits_out: at least 14 bytes.  Returns bit count (includes 2 trailing zeros). */
int  psk_varicode_encode(char c, uint8_t *bits_out);

/* Decode accumulated bits (without trailing 00) to ASCII.  Returns 0 on no match. */
char psk_varicode_decode(const uint8_t *bits, size_t len);
```

### Minimal usage example

```c
#include "psk.h"
#include <stdio.h>

static void on_char(char c, void *ud) { putchar(c); fflush(stdout); }

int main(void) {
    /* TX */
    psk_tx_t *tx = psk_tx_create(8000, 1500.0f, PSK_MODE_BPSK31);
    psk_tx_write(tx, "HELLO WORLD", 11);

    /* RX */
    psk_rx_t *rx = psk_rx_create(8000, 1500.0f, PSK_MODE_BPSK31);
    psk_rx_set_char_callback(rx, on_char, NULL);

    /* Loopback: feed TX output into RX */
    float buf[256];
    while (!psk_tx_idle(tx)) {
        psk_tx_read(tx, buf, 256);
        psk_rx_feed(rx, buf, 256);
    }

    psk_tx_destroy(tx);
    psk_rx_destroy(rx);
    return 0;
}
```

---

## Linux TCP Modem (`psk_tcpmodem`)

`apps/linux_tcpmodem/` builds `psk_tcpmodem`: a Hayes-style modem server that
bridges a TCP connection to a PSK31 audio stream.

### Build

```bash
cmake -B build -S .
cmake --build build --target psk_tcpmodem
```

### Usage

```
psk_tcpmodem [options]

Options:
  --port PORT       TCP listen port (default: 7300)
  --carrier HZ      Carrier frequency in Hz (default: 1500)
  --mode MODE       PSK mode: BPSK31 QPSK31 BPSK63 BPSK125 QPSK63 QPSK125
                    (default: BPSK31)
  --rx-file FILE    Read audio from FILE instead of stdin
  --tx-file FILE    Write audio to FILE instead of stdout
```

By default, audio flows over **stdin** (RX input) and **stdout** (TX output) as raw
float32 PCM at 8000 Hz.  Pipe this to/from your audio hardware using sox or ALSA:

```bash
# Pipe through a soundcard (record from mic, play to speaker)
rec -q -r 8000 -e float -b 32 -c 1 -t raw - | \
    psk_tcpmodem | \
    play -q -r 8000 -e float -b 32 -c 1 -t raw -
```

Or use named files for loopback testing:

```bash
psk_tcpmodem --rx-file /tmp/rx.f32 --tx-file /tmp/tx.f32
```

### Connecting

Once running, open a TCP connection to port 7300 (or your chosen port):

```bash
nc localhost 7300
```

The modem responds immediately:

```
PSK31 MODEM READY
```

The modem starts in **COMMAND mode**.  Type AT commands to configure it, then issue
`ATA` to enter **DATA mode** and begin transmitting/receiving.

---

## AT Command Reference

Commands are case-insensitive.  Terminate each command with `\r` (carriage return).

| Command | Response | Description |
|---------|----------|-------------|
| `AT` | `OK` | Attention — check that the modem is alive |
| `ATZ` | `OK` | Reset to defaults (BPSK31, default carrier) |
| `ATH` or `ATH0` | `NO CARRIER` | Hang up — return to idle/command mode |
| `ATA` | `CONNECT` | Answer/go online — enter DATA mode |
| `AT+MODE=<mode>` | `OK` or `ERROR` | Set PSK mode (see modes table) |
| `AT+FREQ=<hz>` | `OK` or `ERROR` | Set carrier frequency in Hz (100–3500) |
| `AT+MODE?` | `<mode>` then `OK` | Query current mode |
| `AT+FREQ?` | `<hz>` then `OK` | Query current carrier frequency |
| `ATI` | version string then `OK` | Modem identification |

**Mode strings** for `AT+MODE=`: `BPSK31`, `QPSK31`, `BPSK63`, `BPSK125`, `QPSK63`,
`QPSK125` (or numeric index 0–5).

Invalid arguments return `ERROR`.  Both `AT+MODE=` and `AT+FREQ=` reinitialise the
TX and RX contexts immediately; any buffered transmit data is discarded.

### Session walkthrough

```
< PSK31 MODEM READY
> AT
< OK
> AT+MODE=BPSK31
< OK
> AT+FREQ=1500
< OK
> ATA
< CONNECT
> Hello, this is a PSK31 transmission.
  (everything typed is now transmitted as PSK31 audio)
  (received PSK31 text appears on the connection in real time)
```

---

## +++ Escape Sequence

While in DATA mode, send `+++` to return to COMMAND mode without dropping the
audio link.  The escape sequence uses a **guard time** on both sides to prevent
data containing `+++` from accidentally triggering it:

1. **Before**: no characters sent for at least **1 second** (8000 samples)
2. **During**: send exactly three `+` characters
3. **After**: wait at least **1 second** with no further characters

If the guard time conditions are met, the modem responds:

```
OK
```

and returns to COMMAND mode.  Issue `ATA` to re-enter DATA mode.

If the `+++` appears in the middle of a transmission (not surrounded by silence),
it is treated as literal data and transmitted.

---

## ESP32 Modem (`apps/esp32_modem/`)

`apps/esp32_modem/main.c` implements the same Hayes AT interface over UART,
with I2S audio for transmit and receive.  It is structured around four HAL
stubs that you replace with real ESP-IDF calls:

```c
/* Replace these with your board's ESP-IDF I2S / UART implementations */
int  hal_audio_read (float *buf, int frames);   /* read from I2S microphone */
void hal_audio_write(const float *buf, int frames); /* write to I2S speaker */
int  hal_uart_read  (char *buf, int len);       /* read from UART (non-blocking) */
void hal_uart_write (const char *buf, int len); /* write to UART */
```

### Default configuration

```c
#define SAMPLE_RATE    8000
#define CARRIER_HZ     1500.0f
#define DEFAULT_PSK_MODE  PSK_MODE_BPSK31
#define AUDIO_BLK      64      /* samples per tick */
```

### Startup

On power-on, `modem_init_esp32()` creates the TX/RX contexts and sends:

```
PSK31 ESP32 MODEM READY
```

The modem starts in COMMAND mode.  The same AT commands and `+++` escape sequence
described above apply identically over the UART.

### Main loop

```c
void app_main(void) {
    modem_init_esp32();
    while (1) {
        modem_tick();                      /* process one AUDIO_BLK */
        vTaskDelay(pdMS_TO_TICKS(8));      /* ~8 ms @ 8 kHz, 64 samples */
    }
}
```

`modem_tick()` reads one block of UART input, processes any AT commands or data,
reads one block of audio from the I2S mic, feeds it to the decoder, and writes
one block of TX audio to the I2S speaker.  Call it from a FreeRTOS task at a
period equal to `AUDIO_BLK / SAMPLE_RATE` seconds (8 ms for the defaults).

### Build (ESP-IDF)

```bash
cd apps/esp32_modem
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

The ESP32-S3 component CMakeLists must add the `libpsk` sources to its component
source list.  Copy or symlink `src/` and `include/` into the component tree and
add them to `idf_component_register`.

### Resource usage

| Resource | Per context |
|----------|-------------|
| RAM | < 4 KB |
| CPU (BPSK31 at 240 MHz) | < 0.1% |
| FPU required? | No (float only, no double) |
| POSIX required? | No |

---

## Tests

Five CTest cases run automatically with `ctest --test-dir build`:

| Test | What it checks |
|------|---------------|
| `varicode` | Round-trip encode/decode of all 128 ASCII characters |
| `tx` | Spectral purity of TX output (energy outside ±50 Hz < −40 dBc) |
| `loopback` | TX → RX round-trip for BPSK31, BPSK63, BPSK125 |
| `snr` | AWGN tolerance: decodes "TEST" at +20, +10, +5, 0, −5, −10 dB SNR |
| `modes` | All six modes (BPSK/QPSK 31/63/125) loopback |

---

## License

GPL-3.0. Copyright (c) 2025 Jeff Francis N0GQ / Ordo Artificum LLC.

PSK31 algorithm by Peter Martinez G3PLX (public domain).
Varicode table from the original G3PLX PSK31 specification (public domain).

The DSP implementation is original work. Any project that statically links
`libpsk` must also be distributed under GPL-3.0 or a compatible license.

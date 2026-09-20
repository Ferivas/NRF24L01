# AGENTS.md

PlatformIO firmware (ATmega328P Pro Mini + nRF24L01) + hardware docs.

## Layout

- `platformio.ini` — envs `tx_node` (`-DTX_NODE`) and `rx_node` (`-DRX_NODE`), same `src/main.cpp`.
  Upload is serial via Optiboot (`upload_protocol = arduino`, `upload_speed = 57600`);
  pass `--upload-port` explicitly.
- `src/main.cpp` — frames, IEEE CRC32 (nibble table, verified vs `zlib`), RF24
  fragmented transport, PC1 trigger, `SETID`/`SHOWID` serial cmds.
- RF24 resolves to 1.6.x: `begin()` validates CONFIG read-back and only waits
  5 ms, but nRF24L01 needs ~100 ms POR — `setup()` delays 150 ms and retries
  init forever (PB1 alarm + diagnosis) instead of halting.
- `README.md` — wiring, voltage limits, BASEDEV setup (do not relax).
- `DOCS/`, `SCH/` — reference images and BASEDEV PDFs. Binary; do not edit.

Note: Bascom TX/RX programs were removed from history (`git log`); do not reconstruct them unless asked.

## Hardware constraints (from README, do not relax)

- VCC 1.9–3.6 V, 3.3 V nominal. 5 V MCU must use ~2 kΩ series resistors on IO lines.
- CE = output, CSN = SPI SS, IRQ = input.
- Pro Mini (ATmega328P) mapping: PB2=CSN, PB5=SCK, PB3=MOSI, PB4=MISO, PB0=CE, PD2=IRQ.
- BASEDEV at 3.3 V requires: close SB1, fit BU33SD5WG-TR regulator, close SB3, bridge ESP32 TX/RX lines to H3, remove Arduino 0 Ω resistor (disconnects 5 V to its regulator).

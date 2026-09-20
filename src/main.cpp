/*
 * NRF24L01 test link between two Arduino Pro Mini (ATmega328P).
 *
 * Build TX node:  pio run -e tx_node -t upload --upload-port /dev/ttyUSB0
 * Build RX node:  pio run -e rx_node -t upload --upload-port /dev/ttyUSB0
 * (upload serie via Optiboot a 57600 baud; ajusta el puerto segun tu PC)
 *
 * Wiring (see README.md):
 *   PB2 (D10) = CSN, PB5 (D13) = SCK, PB3 (D11) = MOSI,
 *   PB4 (D12) = MISO, PB0 (D8) = CE, PD2 (D2) = IRQ (not used, polled)
 *   PC1 (A1)  = TX trigger input, active falling edge (HIGH -> LOW)
 *   VCC 3.3V nominal (1.9-3.6V). 5V MCU needs ~2k series resistors on IO.
 *
 * Frame formats (ASCII over serial AND over the air, reassembled):
 *   TX: $IDSERIAL,CONTADOR&CRC32[CR][LF]
 *   ACK:$OK,CONTADOR&CRC32[CR][LF]
 *   - IDSERIAL: 10 binary bytes rendered as 20 uppercase hex chars.
 *   - CONTADOR: uint16 decimal ("0".."65535").
 *   - CRC32: IEEE 802.3 of the chars between '$' and '&' (both excluded),
 *     rendered as 8 uppercase hex chars, zero-padded.
 *
 * Air transport: an ASCII DATA frame is up to 38 bytes, but one nRF24L01
 * payload holds max 32 bytes. Frames are therefore fragmented into fixed
 * 32-byte air packets: [seq][part][total][len][data...] with data <= 28 B.
 * DATA frames use total=2, ACK frames total=1. The receiver reassembles
 * by (seq,part) and validates the ASCII frame (incl. CRC32) afterwards.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

// ---------------------------------------------------------------- config

// RF channel 0..125 -> 2400 + CH MHz. Change to move away from WiFi.
static const uint8_t RF_CHANNEL = 76;

// Pipes must match on both nodes (5-byte addresses).
static const uint8_t PIPE_TX2RX[6] = "TXRX1";
static const uint8_t PIPE_RX2TX[6] = "RXTX1";

static const uint8_t PIN_CE = 8;    // PB0
static const uint8_t PIN_CSN = 10;  // PB2 (SS)
static const uint8_t PIN_TRIGGER = A1; // PC1, falling-edge trigger (TX node)
static const uint8_t PIN_SIGNAL = 9;  // PB1, LED externo de estado (el LED
                                       // integrado va a D13 = SCK, no usarlo)

static const unsigned long ACK_TIMEOUT_MS = 1000;
static const uint8_t MAX_ATTEMPTS = 3; // 1 envio + 2 reintentos
static const unsigned long DEBOUNCE_MS = 60;

// Serial ID: 10 bytes stored in EEPROM 0..9 (see note below).
static const int EEPROM_ID_ADDR = 0;
static const uint8_t ID_LEN = 10;

#if !defined(TX_NODE) && !defined(RX_NODE)
#error "Define TX_NODE or RX_NODE (see platformio.ini envs tx_node/rx_node)."
#endif

RF24 radio(PIN_CE, PIN_CSN);

// Forward declarations (order-independent by design)
void handle_serial_cmd(void);
static void radio_retest(void);
static void radio_power_cycle(void);
static void spi_loopback_test(void);
static void signal_blink(uint8_t times);
static void print_sanitized(const char *s);
static void print_hex(const char *buf, size_t len);

// ------------------------------------------------------------ CRC32 (IEEE)

static const uint32_t CRC32_TABLE[16] PROGMEM = {
  0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
  0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
  0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
  0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
};

// Nibble-wise table: ~64 B flash instead of 1 KB. Fine for 40-byte frames.
static uint32_t crc32_update(uint32_t crc, const char *data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = (uint8_t)data[i];
    crc = (crc >> 4) ^ pgm_read_dword(&CRC32_TABLE[(crc ^ b) & 0x0F]);
    crc = (crc >> 4) ^ pgm_read_dword(&CRC32_TABLE[(crc ^ (b >> 4)) & 0x0F]);
  }
  return ~crc;
}

static void to_hex8(uint32_t v, char *out) {
  static const char D[] = "0123456789ABCDEF";
  for (int i = 7; i >= 0; i--) { out[i] = D[v & 0x0F]; v >>= 4; }
  out[8] = '\0';
}

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// ------------------------------------------------------------ serial ID
//
// The spec asks for a 10-byte factory serial from the "Signature/User Row".
// ATmega328P has NO 10-byte user row: its signature row is only 3 device
// bytes (0x1E 0x95 0x0F) + oscillator calibration. So:
//   - primary ID lives in EEPROM bytes 0..9 (survives reflash),
//   - program it once with:  SETID 00112233445566778899
//   - until programmed, a deterministic fallback derived from the real
//     signature bytes is used (and a warning is printed).

static void read_signature(uint8_t *sig, uint8_t n) {
  uint8_t m = n > 5 ? 5 : n;
  for (uint8_t i = 0; i < m; i++) sig[i] = boot_signature_byte_get(i);
  for (uint8_t i = m; i < n; i++) sig[i] = 0;
}

static bool eeprom_id_programmed(void) {
  for (uint8_t i = 0; i < ID_LEN; i++)
    if (EEPROM.read(EEPROM_ID_ADDR + i) != 0xFF) return true;
  return false;
}

static void load_id(uint8_t *id) {
  if (eeprom_id_programmed()) {
    for (uint8_t i = 0; i < ID_LEN; i++) id[i] = EEPROM.read(EEPROM_ID_ADDR + i);
    return;
  }
  // Fallback: signature bytes spread over 10 bytes (deterministic).
  uint8_t sig[5];
  read_signature(sig, sizeof(sig));
  for (uint8_t i = 0; i < ID_LEN; i++) {
    id[i] = (uint8_t)(sig[i % sizeof(sig)] ^ (i * 0x5A) ^ 0xA5);
  }
}

static void id_to_hex(const uint8_t *id, char *out20) {
  static const char D[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < ID_LEN; i++) {
    out20[2 * i] = D[id[i] >> 4];
    out20[2 * i + 1] = D[id[i] & 0x0F];
  }
  out20[20] = '\0';
}

// ------------------------------------------------------------ frame codec

// Builds "$<idhex>,<counter>&<crc>\r\n". Returns length (excl. NUL).
static size_t build_data_frame(const char *idhex, uint16_t counter,
                               char *out, size_t outSize) {
  char body[32];
  int bodyLen = snprintf(body, sizeof(body), "%s,%u", idhex, (unsigned)counter);
  if (bodyLen <= 0) return 0;
  uint32_t crc = crc32_update(0, body, (size_t)bodyLen);
  char hex[9];
  to_hex8(crc, hex);
  return (size_t)snprintf(out, outSize, "$%s&%s\r\n", body, hex);
}

static size_t build_ack_frame(uint16_t counter, char *out, size_t outSize) {
  char body[16];
  int bodyLen = snprintf(body, sizeof(body), "OK,%u", (unsigned)counter);
  if (bodyLen <= 0) return 0;
  uint32_t crc = crc32_update(0, body, (size_t)bodyLen);
  char hex[9];
  to_hex8(crc, hex);
  return (size_t)snprintf(out, outSize, "$%s&%s\r\n", body, hex);
}

// Validates "$body&CRC\r\n" (CRLF optional). On success, body is copied
// (without '$', without "&CRC...") and bodyLen set. Returns true if CRC ok.
static bool split_and_check_crc(const char *frame, char *bodyOut,
                                size_t bodyOutSize, size_t *bodyLenOut) {
  if (!frame || frame[0] != '$') return false;
  const char *amp = strchr(frame + 1, '&');
  if (!amp) return false;
  size_t bodyLen = (size_t)(amp - (frame + 1));
  if (bodyLen == 0 || bodyLen + 1 > bodyOutSize) return false;
  // Expect 8 hex digits after '&' (CRLF after them is optional).
  if (strlen(amp + 1) < 8) return false;
  for (uint8_t i = 0; i < 8; i++)
    if (hexval(amp[1 + i]) < 0) return false;
  uint32_t want = 0;
  for (uint8_t i = 0; i < 8; i++) want = (want << 4) | (uint32_t)hexval(amp[1 + i]);
  uint32_t got = crc32_update(0, frame + 1, bodyLen);
  if (got != want) return false;
  memcpy(bodyOut, frame + 1, bodyLen);
  bodyOut[bodyLen] = '\0';
  if (bodyLenOut) *bodyLenOut = bodyLen;
  return true;
}

// body = "OK,<counter>"
static bool parse_ack_body(const char *body, uint16_t *counterOut) {
  if (strncmp(body, "OK,", 3) != 0) return false;
  char *end = NULL;
  long v = strtol(body + 3, &end, 10);
  if (!end || end == body + 3 || *end != '\0' || v < 0 || v > 65535) return false;
  *counterOut = (uint16_t)v;
  return true;
}

// body = "<idhex>,<counter>"
static bool parse_data_body(const char *body, uint16_t *counterOut) {
  const char *comma = strchr(body, ',');
  if (!comma) return false;
  if ((size_t)(comma - body) != 20) return false; // 10 bytes hex
  for (uint8_t i = 0; i < 20; i++)
    if (hexval(body[i]) < 0) return false;
  char *end = NULL;
  long v = strtol(comma + 1, &end, 10);
  if (!end || end == comma + 1 || *end != '\0' || v < 0 || v > 65535) return false;
  *counterOut = (uint16_t)v;
  return true;
}

// ------------------------------------------------------------ air packets

struct AirPacket {
  uint8_t seq;
  uint8_t part;
  uint8_t total;
  uint8_t len;      // valid bytes in data
  char data[28];    // 4 + 28 = 32 bytes == nRF24 max payload
};
static_assert(sizeof(AirPacket) == 32, "AirPacket must be 32 bytes");

// Splits an ASCII frame into 1..2 air packets and transmits them in order.
static bool air_send(uint8_t seq, const char *frame, size_t frameLen) {
  uint8_t total = (uint8_t)((frameLen + 27) / 28);
  for (uint8_t p = 0; p < total; p++) {
    AirPacket pkt;
    pkt.seq = seq;
    pkt.part = p;
    pkt.total = total;
    size_t off = (size_t)p * 28;
    pkt.len = (uint8_t)((frameLen - off > 28) ? 28 : (frameLen - off));
    memcpy(pkt.data, frame + off, pkt.len);
    if (pkt.len < 28) memset(pkt.data + pkt.len, 0, 28 - pkt.len);
    if (!radio.write(&pkt, sizeof(pkt))) return false;
    delay(5); // spacing between fragments
  }
  return true;
}

// Reassembly buffer for one incoming frame (single peer, stop-and-wait).
struct Reassembler {
  uint8_t seq;
  uint8_t total;
  uint8_t got;      // bitmask of received parts
  char frame[48];   // longest DATA frame is 38 B + margin
  size_t frameLen;

  void reset() { seq = 0; total = 0; got = 0; frameLen = 0; }

  // Returns true when a complete frame is ready (NUL-terminated in frame).
  bool feed(const AirPacket *pkt) {
    if (pkt->total == 0 || pkt->total > 2 || pkt->part >= pkt->total ||
        pkt->len > 28)
      return false;
    if (got == 0 || pkt->seq != seq) { // new frame
      seq = pkt->seq;
      total = pkt->total;
      got = 0;
      frameLen = 0;
    }
    if (pkt->total != total) { reset(); return false; }
    if (got & (1 << pkt->part)) return false; // duplicate fragment
    size_t off = (size_t)pkt->part * 28;
    if (off + pkt->len >= sizeof(frame)) { reset(); return false; }
    memcpy(frame + off, pkt->data, pkt->len);
    frameLen = off + pkt->len;
    got |= (1 << pkt->part);
    if (got == (uint8_t)((1 << total) - 1)) {
      if (frameLen >= sizeof(frame)) { reset(); return false; }
      frame[frameLen] = '\0';
      return true;
    }
    return false;
  }
};

// Imprime un string sustituyendo no-imprimibles por '.' (CR/LF se respetan).
// Evita basura en el monitor si llega un reensamblado corrupto.
static void print_sanitized(const char *s) {
  for (; *s; s++) {
    char c = *s;
    if (c == '\r' || c == '\n' || (c >= 32 && c <= 126)) Serial.print(c);
    else Serial.print('.');
  }
}

static void print_hex(const char *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if ((uint8_t)buf[i] < 0x16) Serial.print('0');
    Serial.print((uint8_t)buf[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
  Serial.println();
}

// ------------------------------------------------------------ serial cmds

static uint8_t g_id[ID_LEN];
static char g_idhex[21];

static void print_id_info(void) {
  uint8_t sig[5];
  read_signature(sig, sizeof(sig));
  Serial.print(F("ID EEPROM : "));
  Serial.println(g_idhex);
  Serial.print(F("Signature : "));
  for (uint8_t i = 0; i < 5; i++) {
    if (sig[i] < 0x16) Serial.print('0');
    Serial.print(sig[i], HEX);
    if (i < 4) Serial.print(' ');
  }
  Serial.println();
  if (!eeprom_id_programmed())
    Serial.println(F("WARN: EEPROM ID sin programar, usando fallback de signature. Use SETID <20 hex>."));
}

static char g_cmd[48];
static uint8_t g_cmdLen = 0;

void handle_serial_cmd(void) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (g_cmdLen == 0) continue;
      g_cmd[g_cmdLen] = '\0';
      if (strncmp(g_cmd, "SETID ", 6) == 0) {
        const char *h = g_cmd + 6;
        if (strlen(h) == 20) {
          bool ok = true;
          uint8_t nid[ID_LEN];
          for (uint8_t i = 0; i < ID_LEN; i++) {
            int hi = hexval(h[2 * i]), lo = hexval(h[2 * i + 1]);
            if (hi < 0 || lo < 0) { ok = false; break; }
            nid[i] = (uint8_t)((hi << 4) | lo);
          }
          if (ok) {
            for (uint8_t i = 0; i < ID_LEN; i++)
              EEPROM.update(EEPROM_ID_ADDR + i, nid[i]);
            memcpy(g_id, nid, ID_LEN);
            id_to_hex(g_id, g_idhex);
            Serial.print(F("ID OK: "));
            Serial.println(g_idhex);
          } else {
            Serial.println(F("ERR: SETID necesita 20 digitos hex, ej: SETID 00112233445566778899"));
          }
        } else {
          Serial.println(F("ERR: SETID necesita 20 digitos hex, ej: SETID 00112233445566778899"));
        }
      } else if (strcmp(g_cmd, "SHOWID") == 0) {
        print_id_info();
      } else if (strcmp(g_cmd, "RADIO") == 0) {
        radio_retest();
      } else if (strcmp(g_cmd, "SPITEST") == 0) {
        spi_loopback_test();
      } else {
        Serial.println(F("cmds: SETID <20hex> | SHOWID | RADIO | SPITEST"));
      }
      g_cmdLen = 0;
    } else if (g_cmdLen < sizeof(g_cmd) - 1) {
      g_cmd[g_cmdLen++] = c;
    }
  }
}

// ------------------------------------------------------------ radio setup

static void radio_common_setup(void) {
  radio.begin();
  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.setPayloadSize(sizeof(AirPacket));
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setCRCLength(RF24_CRC_16);
}

static void radio_retest(void) {
  radio_common_setup();
#ifdef TX_NODE
  radio.openWritingPipe(PIPE_TX2RX);
  radio.openReadingPipe(1, PIPE_RX2TX);
#else
  radio.openWritingPipe(PIPE_RX2TX);
  radio.openReadingPipe(1, PIPE_TX2RX);
#endif
  radio.startListening();
  Serial.print(F("RADIO: "));
  Serial.println(radio.isChipConnected() ? F("ok") : F("FALLO"));
}

// ------------------------------------------------------------ SPI loopback

static void spi_loopback_test(void) {
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  uint8_t r1 = SPI.transfer(0x55);
  uint8_t r2 = SPI.transfer(0xAA);
  SPI.endTransaction();
  Serial.print(F("TX 55 -> RX "));
  Serial.println(r1, HEX);
  Serial.print(F("TX AA -> RX "));
  Serial.println(r2, HEX);
  if (r1 == 0x55 && r2 == 0xAA)
    Serial.println(F("SPI OK: el ATmega transmite/recibe. El fallo esta en el modulo, su alimentacion o su cableado."));
  else
    Serial.println(F("Sin eco: verifica el puente MOSI-MISO y repite. Si con puente sigue sin eco, el problema esta en el ATmega."));
}

static void radio_power_cycle(void) {
  // Pone el modulo en modo power-down y espera para que las
  // lineas SPI queden estables antes de un nuevo begin().
  radio.powerDown();
  digitalWrite(PIN_CE, LOW);
  delay(100); // t_rc: 1.5 ms minimo, 100 ms para estabilidad robusta
}

// ------------------------------------------------------------ TX node

#ifdef TX_NODE
static uint16_t g_counter = 0;
static uint8_t g_seq = 0;
static int g_lastTrig = HIGH;
static unsigned long g_lastEdge = 0;

static void tx_transaction(uint16_t counter) {
  char frame[48];
  size_t flen = build_data_frame(g_idhex, counter, frame, sizeof(frame));
  if (flen == 0) { Serial.println(F("ERR: no se pudo construir trama")); return; }

  bool ok = false;
  for (uint8_t att = 1; att <= MAX_ATTEMPTS && !ok; att++) {
    g_seq++;
    radio.stopListening();
    bool sent = air_send(g_seq, frame, flen);
    radio.startListening();

    Serial.print(F("TX> "));
    Serial.print(frame);
    if (!sent) {
      Serial.println(F("  (write fail link-level)"));
      continue;
    }

    Reassembler ra;
    ra.reset();
    unsigned long t0 = millis();
    bool done = false;
    while (millis() - t0 < ACK_TIMEOUT_MS && !done) {
      if (radio.available()) {
        AirPacket pkt;
        radio.read(&pkt, sizeof(pkt));
        if (ra.feed(&pkt)) {
          Serial.print(F("ACK< "));
          print_sanitized(ra.frame);
          Serial.println();
          char body[40];
          if (split_and_check_crc(ra.frame, body, sizeof(body), NULL)) {
            uint16_t ackCnt = 0;
            if (parse_ack_body(body, &ackCnt) && ackCnt == counter) {
              Serial.print(F("OK cnt="));
              Serial.print(counter);
              Serial.print(F(" intento="));
              Serial.println(att);
              ok = true;
            } else {
              Serial.println(F("  (ACK con contador distinto, se ignora)"));
            }
          } else {
            Serial.println(F("  (ACK con CRC invalido, se ignora)"));
          }
          done = true;
        }
      } else {
        delay(2);
      }
      handle_serial_cmd();
    }
    if (!ok && !done) Serial.println(F("  (timeout 1s sin ACK)"));
    if (!ok && att < MAX_ATTEMPTS) delay(50);
  }

  if (!ok) {
    Serial.print(F("FAIL cnt="));
    Serial.print(counter);
    Serial.println(F(" tras 3 intentos"));
    signal_blink(3);
  } else {
    signal_blink(1);
  }
  g_counter = counter + 1;
}

static void tx_setup(void) {
  pinMode(PIN_TRIGGER, INPUT_PULLUP);
  g_lastTrig = digitalRead(PIN_TRIGGER);
  radio.openWritingPipe(PIPE_TX2RX);
  radio.openReadingPipe(1, PIPE_RX2TX);
  radio.startListening();
  Serial.println(F("Nodo TX listo. Flanco HIGH->LOW en PC1(A1) inicia envio."));
}

static void tx_loop(void) {
  int v = digitalRead(PIN_TRIGGER);
  if (g_lastTrig == HIGH && v == LOW && millis() - g_lastEdge > DEBOUNCE_MS) {
    g_lastEdge = millis();
    tx_transaction(g_counter);
  }
  if (v != g_lastTrig) g_lastTrig = v;
  handle_serial_cmd();
}
#endif

// ------------------------------------------------------------ RX node

#ifdef RX_NODE
static void rx_setup(void) {
  radio.openWritingPipe(PIPE_RX2TX);
  radio.openReadingPipe(1, PIPE_TX2RX);
  radio.startListening();
  Serial.println(F("Nodo RX listo, escuchando..."));
}

static void rx_loop(void) {
  handle_serial_cmd();
  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 5000) {
    lastBeat = millis();
    Serial.println(F("RX vivo, esperando tramas..."));
  }
  if (!radio.available()) return;

  static Reassembler ra;
  AirPacket pkt;
  radio.read(&pkt, sizeof(pkt));
  if (!ra.feed(&pkt)) return;

  Serial.print(F("RX< "));
  print_sanitized(ra.frame);
  Serial.println();
  char body[40];
  uint16_t cnt = 0;
  if (split_and_check_crc(ra.frame, body, sizeof(body), NULL) &&
      parse_data_body(body, &cnt)) {
    Serial.print(F("  (CRC ok, cnt="));
    Serial.print(cnt);
    Serial.println(')');
    char ack[24];
    size_t alen = build_ack_frame(cnt, ack, sizeof(ack));
    static uint8_t ackSeq = 0;
    ackSeq++;
    radio.stopListening();
    bool s = air_send(ackSeq, ack, alen);
    radio.startListening();
    Serial.print(F("ACK> "));
    Serial.print(ack);
    if (!s) Serial.println(F("  (write fail link-level)"));
    signal_blink(1);
  } else {
    Serial.println(F("  (trama invalida: CRC o formato, sin ACK)"));
    Serial.print(F("  hex: "));
    print_hex(ra.frame, ra.frameLen);
  }
}
#endif

// ------------------------------------------------------------ signal

static void signal_blink(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(PIN_SIGNAL, HIGH); delay(120);
    digitalWrite(PIN_SIGNAL, LOW); delay(120);
  }
}

// ------------------------------------------------------------ main

void setup(void) {
  Serial.begin(115200);
  while (!Serial) { /* espera USB-serie en placas con nativo */ }
  delay(300);
  load_id(g_id);
  id_to_hex(g_id, g_idhex);

#if defined(TX_NODE)
  Serial.println(F("== NRF24L01 TX (Pro Mini) =="));
#else
  Serial.println(F("== NRF24L01 RX (Pro Mini) =="));
#endif
  Serial.print(F("RF channel: "));
  Serial.print(2400 + (int)RF_CHANNEL);
  Serial.println(F(" MHz"));
  print_id_info();

  pinMode(PIN_SIGNAL, OUTPUT);
  digitalWrite(PIN_SIGNAL, LOW);

  // POR del nRF24L01: hasta ~100 ms tras energizar (la lib solo espera 5 ms).
  // Se hace fuera del while para que el primer begin() tenga un POR completo.
  delay(150);

  unsigned long lastMsg = 0;
  radio_common_setup();
  if (!radio.isChipConnected()) {
    radio_power_cycle(); // powerDown + CE low + t_rc antes de cada begin()
    radio_common_setup();
  }
  while (!radio.isChipConnected()) {
    radio_power_cycle(); // powerDown + CE low + t_rc antes de cada begin()
    radio_common_setup();
    digitalWrite(PIN_SIGNAL, HIGH); delay(200);
    digitalWrite(PIN_SIGNAL, LOW); delay(200);
    if (millis() - lastMsg > 2000) {
      lastMsg = millis();
      Serial.println(F("ERR: NRF24L01 no inicializa (reintentando)."));
    }
  }
  Serial.println(F("NRF24L01 OK (SPI + registros verificados)."));
#ifdef TX_NODE
  tx_setup();
#else
  rx_setup();
#endif
}

void loop(void) {
#ifdef TX_NODE
  tx_loop();
#else
  rx_loop();
#endif
}

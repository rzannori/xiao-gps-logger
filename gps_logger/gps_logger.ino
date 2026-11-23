/*
 * GPS NMEA Logger - Seeed XIAO nRF52840 (+ Sense opzionale)
 * BN-220 (UART) -> scheda microSD (SPI)
 *
 * Core Arduino: "Seeed nRF52 Boards" (NON la versione mbed)
 *   URL board manager: https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
 *
 * Collegamenti (verifica che corrispondano ai tuoi):
 *   BN-220 TX -> D7 (RX Serial1)   BN-220 RX -> D6 (TX Serial1)
 *   SD  SCK -> D8,  MISO -> D9,  MOSI -> D10,  CS -> SD_CS_PIN
 *
 * LED: verde lampeggia = fix, blu = dati GPS senza fix, rosso fisso = errore SD,
 *      rosso lampeggiante = nessun dato dal GPS
 *
 * All'avvio il GPS (u-blox M8 del BN-220) viene configurato via comandi UBX:
 *   frequenza GPS_RATE_HZ, baud GPS_BAUD_FAST, solo frasi GGA+RMC, modello dinamico "Airborne".
 *   (Il BN-220 non ha memoria flash: per questo lo riconfiguriamo ad ogni accensione.)
 *
 * DEBUG SERIALE (USB, 115200 baud) - comandi da Serial Monitor:
 *   h = aiuto            s = stato ora           e = eco NMEA on/off
 *   r = report periodico on/off                  f = flush forzato
 *   l = elenco file SD   g = riconfigura GPS
 *   p = passthrough USB<->GPS (per u-center), esci con "+++"
 */

#include <SPI.h>
#include <SD.h>

// ---------------- CONFIGURAZIONE ----------------
#define SD_CS_PIN        D2      // <-- pin CS della SD
#define GPS_CONFIGURE    1       // 1 = configura il GPS all'avvio (frequenza, baud, frasi)
#define GPS_RATE_HZ      10      // 1, 5 o 10 posizioni al secondo (se 10 da' problemi prova 5)
#define GPS_BAUD_DEFAULT 9600    // baud di fabbrica del BN-220
#define GPS_BAUD_FAST    38400   // baud dopo la configurazione (sufficiente per GGA+RMC a 10 Hz)
#define GPS_DYN_MODEL    8       // 0 = portatile, 7 = aereo <2g, 8 = aereo <4g (acrobazia)
#define GPS_GSA_GSV      0       // 1 = aggiunge GSA/GSV (dettaglio satelliti) 1 volta al secondo
#define FLUSH_INTERVAL   1000    // ms tra un salvataggio fisico su SD e l'altro (500-1000)
#define LOG_ONLY_VALID   1       // 1 = scrive solo frasi con checksum corretto
#define LOG_EXTRA        1       // 1 = riga $PXIAO ogni secondo (temperatura CPU, +IMU)
#define USE_IMU          0       // 1 = solo XIAO nRF52840 *Sense*

#define DEBUG            1       // 0 = nessun messaggio su USB (versione "da campo")
#define DEBUG_WAIT_MS    3000    // all'avvio attende il Serial Monitor fino a X ms
#define REPORT_INTERVAL  5000    // ms tra i report di stato automatici
// ------------------------------------------------

#if DEBUG
  #define DBG(...)    do { if (Serial) Serial.print(__VA_ARGS__); } while (0)
  #define DBGLN(...)  do { if (Serial) Serial.println(__VA_ARGS__); } while (0)
  #define DBGF(...)   do { if (Serial) Serial.printf(__VA_ARGS__); } while (0)
#else
  #define DBG(...)
  #define DBGLN(...)
  #define DBGF(...)
#endif

#if USE_IMU
  #include <LSM6DS3.h>           // libreria "Seeed Arduino LSM6DS3"
  #include <Wire.h>
  LSM6DS3 imu(I2C_MODE, 0x6A);
  bool imuOk = false;
#endif

File logFile;
char logName[13] = "-";
bool sdOk = false;
char lineBuf[128];
uint8_t lineLen = 0;
uint32_t lastFlush = 0, lastExtra = 0, lastGpsData = 0, lastReport = 0;

// Stato GPS (da GGA)
bool gpsFix = false;
int  gpsSats = 0;
char gpsHdop[8] = "-";
char gpsTime[12] = "-";

// Contatori diagnostici
struct {
  uint32_t gpsBytes, sentences, csErrors, overflows, ggaCount;
  uint32_t linesWritten, bytesWritten, writeErrors, sdReinits;
  uint32_t maxFlushMs;
  uint32_t bytesLastReport, ggaLastReport;
} st = {0};

bool echoNmea = false;     // eco NMEA su USB (all'inizio spento: troppo testo)
bool autoReport = true;

inline void led(int pin, bool on) { digitalWrite(pin, on ? LOW : HIGH); }   // LED attivi bassi

bool nmeaChecksumOk(const char *s) {
  if (s[0] != '$') return false;
  const char *star = strchr(s, '*');
  if (!star || strlen(star) < 3) return false;
  uint8_t cs = 0;
  for (const char *p = s + 1; p < star; p++) cs ^= (uint8_t)*p;
  return cs == (uint8_t)strtol(star + 1, NULL, 16);
}

// Copia il campo n (0 = "$GxGGA") di una frase NMEA
void nmeaField(const char *s, int n, char *out, size_t outLen) {
  int f = 0;
  while (*s && f < n) { if (*s++ == ',') f++; }
  size_t i = 0;
  while (*s && *s != ',' && *s != '*' && i < outLen - 1) out[i++] = *s++;
  out[i] = '\0';
}

void parseGGA(const char *s) {
  if (strncmp(s + 3, "GGA", 3) != 0) return;
  st.ggaCount++;
  char buf[12];
  nmeaField(s, 1, buf, sizeof(buf));
  if (strlen(buf) >= 6) snprintf(gpsTime, sizeof(gpsTime), "%.2s:%.2s:%.2s", buf, buf + 2, buf + 4);
  nmeaField(s, 6, buf, sizeof(buf)); gpsFix = (buf[0] >= '1' && buf[0] <= '9');
  nmeaField(s, 7, buf, sizeof(buf)); gpsSats = atoi(buf);
  nmeaField(s, 8, gpsHdop, sizeof(gpsHdop));
  if (!gpsHdop[0]) strcpy(gpsHdop, "-");
}

// ---------------- CONFIGURAZIONE GPS (protocollo UBX u-blox) ----------------
uint32_t gpsBaud = GPS_BAUD_DEFAULT;

void ubxSend(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t head[4] = { cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t ckA = 0, ckB = 0;
  Serial1.write(0xB5); Serial1.write(0x62);
  for (int i = 0; i < 4; i++)   { Serial1.write(head[i]);    ckA += head[i];    ckB += ckA; }
  for (int i = 0; i < len; i++) { Serial1.write(payload[i]); ckA += payload[i]; ckB += ckA; }
  Serial1.write(ckA); Serial1.write(ckB);
  Serial1.flush();
  delay(60);
}

// CFG-PRT: imposta il baud della UART1 del GPS (ingresso UBX+NMEA, uscita solo NMEA)
void ubxSetBaud(uint32_t baud) {
  uint8_t p[20] = {0};
  p[0] = 1;                       // porta UART1
  p[4] = 0xD0; p[5] = 0x08;       // 8 bit, nessuna parita', 1 stop
  p[8] = baud; p[9] = baud >> 8; p[10] = baud >> 16; p[11] = baud >> 24;
  p[12] = 0x03;                   // protocolli in ingresso: UBX + NMEA
  p[14] = 0x02;                   // protocolli in uscita: solo NMEA
  ubxSend(0x06, 0x00, p, sizeof(p));
}

// CFG-MSG: quante volte (ogni N soluzioni) inviare una frase NMEA. 0 = disattivata
void ubxSetNmeaRate(uint8_t nmeaId, uint8_t rate) {
  uint8_t p[3] = { 0xF0, nmeaId, rate };
  ubxSend(0x06, 0x01, p, sizeof(p));
}

// CFG-RATE: intervallo tra le misure in ms
void ubxSetMeasRate(uint16_t ms) {
  uint8_t p[6] = { (uint8_t)(ms & 0xFF), (uint8_t)(ms >> 8), 1, 0, 1, 0 };
  ubxSend(0x06, 0x08, p, sizeof(p));
}

// CFG-NAV5: modello dinamico (solo quel campo, maschera 0x0001)
void ubxSetDynModel(uint8_t model) {
  uint8_t p[36] = {0};
  p[0] = 0x01;
  p[2] = model;
  ubxSend(0x06, 0x24, p, sizeof(p));
}

void configureGPS() {
  DBGF("[GPS] Configurazione: %d Hz, %lu baud, modello dinamico %d\n",
       GPS_RATE_HZ, (unsigned long)GPS_BAUD_FAST, GPS_DYN_MODEL);
  // Il GPS potrebbe essere ancora alla velocita' impostata in precedenza (se e' rimasto
  // alimentato dal supercondensatore): mando il cambio baud a tutte le velocita' comuni.
  const uint32_t bauds[] = { 9600, 38400, 57600, 115200 };
  for (uint32_t b : bauds) {
    Serial1.begin(b);
    delay(20);
    ubxSetBaud(GPS_BAUD_FAST);
    Serial1.end();
  }
  gpsBaud = GPS_BAUD_FAST;
  Serial1.begin(gpsBaud);
  delay(100);

  // Frasi NMEA: id 0=GGA 1=GLL 2=GSA 3=GSV 4=RMC 5=VTG
  ubxSetNmeaRate(0x00, 1);                                   // GGA: posizione, quota, satelliti
  ubxSetNmeaRate(0x04, 1);                                   // RMC: posizione, velocita', rotta, data
  ubxSetNmeaRate(0x01, 0);                                   // GLL off
  ubxSetNmeaRate(0x05, 0);                                   // VTG off (velocita' gia' in RMC)
  ubxSetNmeaRate(0x02, GPS_GSA_GSV ? GPS_RATE_HZ : 0);       // GSA
  ubxSetNmeaRate(0x03, GPS_GSA_GSV ? GPS_RATE_HZ : 0);       // GSV
  ubxSetDynModel(GPS_DYN_MODEL);
  ubxSetMeasRate(1000 / GPS_RATE_HZ);
  DBGLN("[GPS] Comandi inviati (verifica 'Frequenza GGA' nel report)");
}

bool openNewLogFile() {
  for (int i = 0; i < 1000; i++) {
    snprintf(logName, sizeof(logName), "GPS%03d.TXT", i);
    if (!SD.exists(logName)) {
      logFile = SD.open(logName, FILE_WRITE);
      if (logFile) { DBGF("[SD] Log su: %s\n", logName); return true; }
      DBGF("[SD] ERRORE: impossibile creare %s\n", logName);
      return false;
    }
  }
  DBGLN("[SD] ERRORE: GPS000..GPS999 gia' tutti presenti");
  return false;
}

bool initSD() {
  DBGF("[SD] Inizializzazione (CS = D%d)... ", SD_CS_PIN);
  SD.end();
  if (!SD.begin(SD_CS_PIN)) {
    DBGLN("FALLITA");
    DBGLN("[SD]  -> controlla: cablaggio SCK/MISO/MOSI/CS, alimentazione modulo (3V3 o 5V?),");
    DBGLN("[SD]     scheda inserita, formattata FAT32, pin SD_CS_PIN corretto");
    return false;
  }
  DBGLN("OK");
  return openNewLogFile();
}

void writeLine(const char *s) {
  if (!sdOk) return;
  size_t n = logFile.println(s);
  if (n == 0) {
    st.writeErrors++;
    DBGLN("[SD] ERRORE scrittura (scheda rimossa o piena?) - riprovo tra 5 s");
    logFile.close();
    sdOk = false;
    led(LED_RED, true);
    return;
  }
  st.linesWritten++;
  st.bytesWritten += n;
}

void printStatus() {
#if DEBUG
  if (!Serial) return;
  uint32_t now = millis();
  uint32_t dt = max(1UL, (unsigned long)(now - lastReport));
  uint32_t bps = (st.gpsBytes - st.bytesLastReport) * 1000UL / dt;
  float ggaHz = (st.ggaCount - st.ggaLastReport) * 1000.0f / dt;
  Serial.println(F("---------------- STATO ----------------"));
  Serial.printf("Uptime         : %lu s\n", (unsigned long)(now / 1000));
  Serial.printf("GPS            : %s  (%lu byte/s, ultimo dato %lu ms fa)\n",
                (now - lastGpsData) < 2000 ? "riceve dati" : "NESSUN DATO!",
                (unsigned long)bps, (unsigned long)(now - lastGpsData));
  Serial.printf("Frequenza GGA  : %.1f Hz (attesa %d Hz), seriale GPS %lu baud\n",
                ggaHz, GPS_CONFIGURE ? GPS_RATE_HZ : 1, (unsigned long)gpsBaud);
  Serial.printf("Fix / satelliti: %s / %d   HDOP %s   ora UTC %s\n",
                gpsFix ? "SI" : "no", gpsSats, gpsHdop, gpsTime);
  Serial.printf("Frasi NMEA     : %lu valide, %lu checksum errati, %lu troppo lunghe\n",
                (unsigned long)st.sentences, (unsigned long)st.csErrors, (unsigned long)st.overflows);
  Serial.printf("SD             : %s  file %s  (%lu righe, %lu byte)\n",
                sdOk ? "OK" : "ERRORE", logName,
                (unsigned long)st.linesWritten, (unsigned long)st.bytesWritten);
  Serial.printf("Errori SD      : %lu scritture fallite, %lu reinizializzazioni, flush max %lu ms\n",
                (unsigned long)st.writeErrors, (unsigned long)st.sdReinits, (unsigned long)st.maxFlushMs);
  Serial.printf("Temperatura CPU: %.1f C\n", readCPUTemperature());
#if USE_IMU
  Serial.printf("IMU            : %s\n", imuOk ? "OK" : "non trovata");
#endif
  if (bps == 0 && now > 3000)
    Serial.println(F("!! Nessun byte dal GPS: controlla TX del BN-220 -> D7, alimentazione, baud"));
  else if (GPS_CONFIGURE && now > 10000 && ggaHz < GPS_RATE_HZ * 0.8f)
    Serial.println(F("!! Frequenza piu' bassa del previsto: configurazione non riuscita (prova 'g') o dati persi"));
  else if (st.csErrors > st.sentences / 10 && st.sentences > 20)
    Serial.println(F("!! Molti checksum errati: baud rate sbagliato o disturbi sulla linea"));
  Serial.println(F("---------------------------------------"));
  st.bytesLastReport = st.gpsBytes;
  st.ggaLastReport = st.ggaCount;
#endif
  lastReport = millis();
}

void listFiles() {
#if DEBUG
  if (!sdOk) { DBGLN("[SD] Scheda non disponibile"); return; }
  File root = SD.open("/");
  DBGLN("[SD] File sulla scheda:");
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    DBGF("   %-13s %10lu byte\n", f.name(), (unsigned long)f.size());
    f.close();
  }
  root.close();
#endif
}

// Ponte trasparente USB <-> GPS: permette di usare u-center per configurare il BN-220
void passthrough() {
  DBGLN("[PT] Passthrough attivo. Logging sospeso. Scrivi +++ per uscire.");
  if (sdOk) logFile.flush();
  uint8_t plus = 0;
  while (true) {
    while (Serial1.available()) Serial.write(Serial1.read());
    while (Serial.available()) {
      char c = Serial.read();
      plus = (c == '+') ? plus + 1 : 0;
      if (plus >= 3) { DBGLN("\n[PT] Passthrough terminato."); return; }
      Serial1.write(c);
    }
    yield();
  }
}

void handleCommands() {
#if DEBUG
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'h': case '?':
        DBGLN("Comandi: s=stato  e=eco NMEA  r=report auto  f=flush  l=file SD  g=riconfigura GPS  p=passthrough GPS");
        break;
      case 's': printStatus(); break;
      case 'e': echoNmea = !echoNmea;  DBGF("[CMD] Eco NMEA %s\n", echoNmea ? "ON" : "OFF"); break;
      case 'r': autoReport = !autoReport; DBGF("[CMD] Report automatico %s\n", autoReport ? "ON" : "OFF"); break;
      case 'f': if (sdOk) { logFile.flush(); DBGLN("[CMD] Flush eseguito"); } break;
      case 'l': listFiles(); break;
      case 'g': configureGPS(); break;
      case 'p': passthrough(); break;
      default: break;   // ignora \r \n ecc.
    }
  }
#endif
}

void setup() {
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT);
  led(LED_RED, false); led(LED_GREEN, false); led(LED_BLUE, false);

  Serial.begin(115200);
#if DEBUG
  // Attende (al massimo DEBUG_WAIT_MS) che si apra il Serial Monitor, cosi' non perdi i messaggi d'avvio.
  // Senza PC collegato parte comunque dopo il timeout.
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < DEBUG_WAIT_MS) { led(LED_BLUE, (millis() / 100) % 2); delay(10); }
  led(LED_BLUE, false);
#endif
  DBGLN("\n=== GPS NMEA Logger - XIAO nRF52840 ===");
  DBGF("Compilato: %s %s\n", __DATE__, __TIME__);
  DBGF("Config: SD CS=D%d, flush %d ms, solo validi=%d, extra=%d, IMU=%d\n",
       SD_CS_PIN, FLUSH_INTERVAL, LOG_ONLY_VALID, LOG_EXTRA, USE_IMU);

#if GPS_CONFIGURE
  configureGPS();
#else
  Serial1.begin(GPS_BAUD_DEFAULT);
#endif
  sdOk = initSD();
  led(LED_RED, !sdOk);

#if USE_IMU
  imuOk = (imu.begin() == 0);
  DBGLN(imuOk ? "[IMU] OK" : "[IMU] non trovata (e' davvero una versione Sense?)");
#endif
  DBGLN("Scrivi 'h' per i comandi.\n");
  lastReport = millis();
}

void loop() {
  // 1) Legge il GPS e scrive riga per riga
  while (Serial1.available()) {
    char c = Serial1.read();
    st.gpsBytes++;
    lastGpsData = millis();
    if (c == '\r') continue;
    if (c == '$') lineLen = 0;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 6) {
        bool ok = nmeaChecksumOk(lineBuf);
        if (ok) { st.sentences++; parseGGA(lineBuf); }
        else    { st.csErrors++; if (echoNmea) DBGF("[CS ERR] %s\n", lineBuf); }
        if (ok || !LOG_ONLY_VALID) {
          writeLine(lineBuf);
          if (echoNmea && ok) DBGLN(lineBuf);
        }
      }
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      st.overflows++;
      lineLen = 0;
    }
  }

  uint32_t now = millis();

#if LOG_EXTRA
  // 2) Riga con i sensori della scheda, formato NMEA proprietario
  if (now - lastExtra >= 1000) {
    lastExtra = now;
    char extra[120];
    int n = snprintf(extra, sizeof(extra), "$PXIAO,%lu,%.1f", (unsigned long)now, readCPUTemperature());
  #if USE_IMU
    if (imuOk)
      n += snprintf(extra + n, sizeof(extra) - n, ",%.3f,%.3f,%.3f,%.1f,%.1f,%.1f",
                    imu.readFloatAccelX(), imu.readFloatAccelY(), imu.readFloatAccelZ(),
                    imu.readFloatGyroX(), imu.readFloatGyroY(), imu.readFloatGyroZ());
  #endif
    uint8_t cs = 0;
    for (int i = 1; i < n; i++) cs ^= (uint8_t)extra[i];
    snprintf(extra + n, sizeof(extra) - n, "*%02X", cs);
    writeLine(extra);
  }
#endif

  // 3) Flush periodico (misura anche quanto dura: se e' lento la SD e' scarsa)
  if (sdOk && now - lastFlush >= FLUSH_INTERVAL) {
    lastFlush = now;
    uint32_t t = millis();
    logFile.flush();
    t = millis() - t;
    if (t > st.maxFlushMs) st.maxFlushMs = t;
    if (t > 200) DBGF("[SD] Attenzione: flush lento (%lu ms)\n", (unsigned long)t);
  }

  // 4) Se la SD e' in errore, riprova ogni 5 s
  static uint32_t lastRetry = 0;
  if (!sdOk && now - lastRetry >= 5000) {
    lastRetry = now;
    st.sdReinits++;
    sdOk = initSD();
    led(LED_RED, !sdOk);
  }

  // 5) Debug: comandi e report periodico
  handleCommands();
  if (DEBUG && autoReport && now - lastReport >= REPORT_INTERVAL) printStatus();

  // 6) LED di stato
  bool gpsAlive = (now - lastGpsData) < 2000;
  bool blink = (now % 1000) < 100;
  led(LED_GREEN, gpsAlive && gpsFix && blink);
  led(LED_BLUE,  gpsAlive && !gpsFix && blink);
  if (sdOk) led(LED_RED, !gpsAlive && (now % 500) < 250);   // rosso lampeggiante = GPS muto
}

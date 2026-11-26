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
 * FILE: una cartella per giorno e un file per accensione, con data e ora locali del GPS:
 *   /20261126/192103.TXT  (= 26 novembre 2026, registrazione iniziata alle 19:21:03)
 *   Il file viene creato appena il GPS conosce la data (di solito pochi secondi dopo l'accensione).
 *   Se la data non arriva entro NODATE_TIMEOUT_S, si registra comunque su GPS000.TXT, GPS001.TXT...
 *
 * LED: verde lampeggia = fix, blu = dati GPS senza fix, rosso fisso = errore SD,
 *      rosso lampeggiante = nessun dato dal GPS
 *
 * All'avvio il GPS (u-blox M8 del BN-220) viene configurato via comandi UBX:
 *   frequenza GPS_RATE_HZ, baud GPS_BAUD_FAST, frasi RMC+GNS+VTG, modello dinamico "Airborne".
 *   RMC+GNS+VTG e' la combinazione letta da RZ Flight Dashboard (stessa dell'SM GPS).
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
#define GPS_BAUD_FAST    38400   // baud dopo la configurazione (RMC+GNS+VTG a 10 Hz ~ 50% della banda)
#define GPS_DYN_MODEL    8       // 0 = portatile, 7 = aereo <2g, 8 = aereo <4g (acrobazia)
#define GPS_GSA_GSV      0       // 1 = aggiunge GSA/GSV (dettaglio satelliti) 1 volta al secondo
#define TZ_OFFSET_H      1       // fuso orario rispetto a UTC (Italia = 1)
#define TZ_EU_DST        1       // 1 = applica l'ora legale europea (ultima dom. di marzo -> ultima dom. di ottobre)
#define NODATE_TIMEOUT_S 120     // se il GPS non da' la data entro X s, usa i nomi GPS000.TXT...
#define FILE_TIMESTAMP   1       // 1 = data/ora del file su SD (visibile in Esplora risorse). Se non compila metti 0
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
char logName[24] = "-";
bool sdOk = false;         // scheda montata
bool fileOpen = false;     // file di log aperto
char lineBuf[128];
uint8_t lineLen = 0;
uint32_t lastFlush = 0, lastExtra = 0, lastGpsData = 0, lastReport = 0;

// Stato GPS (fix e ora da RMC, satelliti e HDOP da GNS o GGA)
bool gpsFix = false;
int  gpsSats = 0;
char gpsHdop[8] = "-";
char gpsTime[12] = "-";

// Data e ora UTC dall'ultima RMC (valide anche prima del fix, appena il GPS conosce l'ora)
bool gpsDateOk = false;
int  utcY, utcMo, utcD, utcH, utcMi, utcS;

// Contatori diagnostici
struct {
  uint32_t gpsBytes, sentences, csErrors, overflows, rmcCount;
  uint32_t linesWritten, bytesWritten, writeErrors, sdReinits;
  uint32_t maxFlushMs;
  uint32_t bytesLastReport, rmcLastReport;
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

// Estrae dalle frasi NMEA i dati per il report di stato e i LED
void parseNmea(const char *s) {
  const char *type = s + 3;                  // salta "$GN" / "$GP"
  char buf[12];
  if (strncmp(type, "RMC", 3) == 0) {        // $GxRMC,hhmmss.ss,A/V,...
    st.rmcCount++;
    nmeaField(s, 1, buf, sizeof(buf));
    if (strlen(buf) >= 6) snprintf(gpsTime, sizeof(gpsTime), "%.2s:%.2s:%.2s", buf, buf + 2, buf + 4);
    nmeaField(s, 2, buf, sizeof(buf));
    gpsFix = (buf[0] == 'A');
    char tm[12], dt[8];
    nmeaField(s, 1, tm, sizeof(tm));
    nmeaField(s, 9, dt, sizeof(dt));               // ddmmyy
    if (strlen(tm) >= 6 && strlen(dt) == 6) {
      int yy = (dt[4] - '0') * 10 + (dt[5] - '0');
      if (yy >= 24 && yy <= 79) {                  // scarta date di default (es. 2080) prima della sincronizzazione
        utcD  = (dt[0] - '0') * 10 + (dt[1] - '0');
        utcMo = (dt[2] - '0') * 10 + (dt[3] - '0');
        utcY  = 2000 + yy;
        utcH  = (tm[0] - '0') * 10 + (tm[1] - '0');
        utcMi = (tm[2] - '0') * 10 + (tm[3] - '0');
        utcS  = (tm[4] - '0') * 10 + (tm[5] - '0');
        gpsDateOk = true;
      }
    }
  } else if (strncmp(type, "GNS", 3) == 0 || strncmp(type, "GGA", 3) == 0) {
    // GNS e GGA hanno satelliti e HDOP negli stessi campi (7 e 8)
    nmeaField(s, 7, buf, sizeof(buf)); gpsSats = atoi(buf);
    nmeaField(s, 8, gpsHdop, sizeof(gpsHdop));
    if (!gpsHdop[0]) strcpy(gpsHdop, "-");
  }
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

  // Frasi NMEA: id 0=GGA 1=GLL 2=GSA 3=GSV 4=RMC 5=VTG 0x0D=GNS
  // RMC+GNS+VTG = formato letto da RZ Flight Dashboard (quota solo da GNS, rotta da VTG)
  ubxSetNmeaRate(0x04, 1);                                   // RMC: ora, data, posizione, validita' fix
  ubxSetNmeaRate(0x0D, 1);                                   // GNS: quota, satelliti, HDOP
  ubxSetNmeaRate(0x05, 1);                                   // VTG: velocita' e rotta
  ubxSetNmeaRate(0x00, 0);                                   // GGA off (stessi dati di GNS)
  ubxSetNmeaRate(0x01, 0);                                   // GLL off
  ubxSetNmeaRate(0x02, GPS_GSA_GSV ? GPS_RATE_HZ : 0);       // GSA
  ubxSetNmeaRate(0x03, GPS_GSA_GSV ? GPS_RATE_HZ : 0);       // GSV
  ubxSetDynModel(GPS_DYN_MODEL);
  ubxSetMeasRate(1000 / GPS_RATE_HZ);
  DBGLN("[GPS] Comandi inviati (verifica 'Frequenza RMC' nel report)");
}

// ---------------- DATA/ORA LOCALE ----------------
// Giorni dal 1/1/1970 <-> data civile (algoritmo di H. Hinnant)
long daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long)doe - 719468;
}
void civilFromDays(long z, int &y, int &m, int &d) {
  z += 719468;
  long era = (z >= 0 ? z : z - 146096) / 146097;
  unsigned doe = (unsigned)(z - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp < 10 ? mp + 3 : mp - 9;
  y = (int)(yoe + era * 400) + (m <= 2);
}
long lastSunday(int y, int m) {                  // giorno (epoch) dell'ultima domenica del mese (marzo/ottobre: 31 giorni)
  long last = daysFromCivil(y, m, 31);
  return last - ((last + 4) % 7);                // 1/1/1970 era giovedi' (4); 0 = domenica
}
// Converte l'ora UTC del GPS in ora locale (fuso + ora legale europea)
void localDateTime(int &y, int &mo, int &d, int &h, int &mi, int &se) {
  int64_t t = (int64_t)daysFromCivil(utcY, utcMo, utcD) * 86400 + utcH * 3600L + utcMi * 60L + utcS;
  int64_t off = TZ_OFFSET_H * 3600L;
#if TZ_EU_DST
  int64_t dstStart = (int64_t)lastSunday(utcY, 3) * 86400 + 3600;    // 01:00 UTC
  int64_t dstEnd   = (int64_t)lastSunday(utcY, 10) * 86400 + 3600;
  if (t >= dstStart && t < dstEnd) off += 3600;
#endif
  t += off;
  long days = (long)(t / 86400);
  long sec  = (long)(t % 86400);
  civilFromDays(days, y, mo, d);
  h = sec / 3600; mi = (sec / 60) % 60; se = sec % 60;
}

#if FILE_TIMESTAMP
// Data/ora scritte nella directory della SD (creazione e ultima modifica del file)
void fatDateTime(uint16_t *date, uint16_t *time) {
  int y = 2000, mo = 1, d = 1, h = 0, mi = 0, se = 0;
  if (gpsDateOk) localDateTime(y, mo, d, h, mi, se);
  *date = (uint16_t)(((y - 1980) << 9) | (mo << 5) | d);
  *time = (uint16_t)((h << 11) | (mi << 5) | (se >> 1));
}
#endif

// ---------------- FILE DI LOG ----------------
bool openLogFile(const char *path) {
  logFile = SD.open(path, FILE_WRITE);
  if (!logFile) { DBGF("[SD] ERRORE: impossibile creare %s\n", path); return false; }
  strncpy(logName, path, sizeof(logName) - 1);
  DBGF("[SD] Log su: %s\n", logName);
  return true;
}

// /AAAAMMGG/hhmmss.TXT con data e ora locali
bool openDatedLogFile() {
  int y, mo, d, h, mi, se;
  localDateTime(y, mo, d, h, mi, se);
  char dir[10], path[24];
  snprintf(dir, sizeof(dir), "%04d%02d%02d", y, mo, d);
  if (!SD.exists(dir) && !SD.mkdir(dir)) { DBGF("[SD] ERRORE: impossibile creare la cartella %s\n", dir); return false; }
  snprintf(path, sizeof(path), "%s/%02d%02d%02d.TXT", dir, h, mi, se);
  for (char c = 'A'; SD.exists(path) && c <= 'Z'; c++)        // gia' esistente: 192103A.TXT, 192103B.TXT...
    snprintf(path, sizeof(path), "%s/%02d%02d%02d%c.TXT", dir, h, mi, se, c);
  return openLogFile(path);
}

// GPS000.TXT, GPS001.TXT... (solo se il GPS non fornisce la data)
bool openNumberedLogFile() {
  char path[16];
  for (int i = 0; i < 1000; i++) {
    snprintf(path, sizeof(path), "GPS%03d.TXT", i);
    if (!SD.exists(path)) return openLogFile(path);
  }
  DBGLN("[SD] ERRORE: GPS000..GPS999 gia' tutti presenti");
  return false;
}

// Apre il file quando la data e' nota (o, scaduto il tempo, con nome numerato)
void tryOpenLogFile() {
  if (!sdOk || fileOpen) return;
  if (gpsDateOk)                                     fileOpen = openDatedLogFile();
  else if (millis() > NODATE_TIMEOUT_S * 1000UL) {
    DBGLN("[SD] Data GPS non disponibile: uso un nome numerato");
    fileOpen = openNumberedLogFile();
  } else return;
  if (!fileOpen) { sdOk = false; led(LED_RED, true); }
}

bool initSD() {
  DBGF("[SD] Inizializzazione (CS = D%d)... ", SD_CS_PIN);
  SD.end();
  fileOpen = false;
  if (!SD.begin(SD_CS_PIN)) {
    DBGLN("FALLITA");
    DBGLN("[SD]  -> controlla: cablaggio SCK/MISO/MOSI/CS, alimentazione modulo (3V3 o 5V?),");
    DBGLN("[SD]     scheda inserita, formattata FAT32, pin SD_CS_PIN corretto");
    return false;
  }
  DBGLN(gpsDateOk ? "OK" : "OK (il file verra' creato appena il GPS conosce la data)");
  return true;
}

void writeLine(const char *s) {
  if (!sdOk || !fileOpen) return;
  size_t n = logFile.println(s);
  if (n == 0) {
    st.writeErrors++;
    DBGLN("[SD] ERRORE scrittura (scheda rimossa o piena?) - riprovo tra 5 s");
    logFile.close();
    fileOpen = false;
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
  float rmcHz = (st.rmcCount - st.rmcLastReport) * 1000.0f / dt;
  Serial.println(F("---------------- STATO ----------------"));
  Serial.printf("Uptime         : %lu s\n", (unsigned long)(now / 1000));
  Serial.printf("GPS            : %s  (%lu byte/s, ultimo dato %lu ms fa)\n",
                (now - lastGpsData) < 2000 ? "riceve dati" : "NESSUN DATO!",
                (unsigned long)bps, (unsigned long)(now - lastGpsData));
  Serial.printf("Frequenza RMC  : %.1f Hz (attesa %d Hz), seriale GPS %lu baud\n",
                rmcHz, GPS_CONFIGURE ? GPS_RATE_HZ : 1, (unsigned long)gpsBaud);
  Serial.printf("Fix / satelliti: %s / %d   HDOP %s   ora UTC %s\n",
                gpsFix ? "SI" : "no", gpsSats, gpsHdop, gpsTime);
  Serial.printf("Frasi NMEA     : %lu valide, %lu checksum errati, %lu troppo lunghe\n",
                (unsigned long)st.sentences, (unsigned long)st.csErrors, (unsigned long)st.overflows);
  Serial.printf("SD             : %s  file %s  (%lu righe, %lu byte)\n",
                !sdOk ? "ERRORE" : (fileOpen ? "OK" : "in attesa della data GPS"), logName,
                (unsigned long)st.linesWritten, (unsigned long)st.bytesWritten);
  Serial.printf("Errori SD      : %lu scritture fallite, %lu reinizializzazioni, flush max %lu ms\n",
                (unsigned long)st.writeErrors, (unsigned long)st.sdReinits, (unsigned long)st.maxFlushMs);
  Serial.printf("Temperatura CPU: %.1f C\n", readCPUTemperature());
#if USE_IMU
  Serial.printf("IMU            : %s\n", imuOk ? "OK" : "non trovata");
#endif
  if (bps == 0 && now > 3000)
    Serial.println(F("!! Nessun byte dal GPS: controlla TX del BN-220 -> D7, alimentazione, baud"));
  else if (GPS_CONFIGURE && now > 10000 && rmcHz < GPS_RATE_HZ * 0.8f)
    Serial.println(F("!! Frequenza piu' bassa del previsto: configurazione non riuscita (prova 'g') o dati persi"));
  else if (st.csErrors > st.sentences / 10 && st.sentences > 20)
    Serial.println(F("!! Molti checksum errati: baud rate sbagliato o disturbi sulla linea"));
  Serial.println(F("---------------------------------------"));
  st.bytesLastReport = st.gpsBytes;
  st.rmcLastReport = st.rmcCount;
#endif
  lastReport = millis();
}

void listDir(File dir, int depth) {
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    if (f.isDirectory()) {
      DBGF("   %*s%s/\n", depth * 3, "", f.name());
      if (depth < 1) listDir(f, depth + 1);
    } else {
      DBGF("   %*s%-13s %10lu byte\n", depth * 3, "", f.name(), (unsigned long)f.size());
    }
    f.close();
  }
}

void listFiles() {
#if DEBUG
  if (!sdOk) { DBGLN("[SD] Scheda non disponibile"); return; }
  if (fileOpen) logFile.flush();
  File root = SD.open("/");
  DBGLN("[SD] File sulla scheda:");
  listDir(root, 0);
  root.close();
#endif
}

// Ponte trasparente USB <-> GPS: permette di usare u-center per configurare il BN-220
void passthrough() {
  DBGLN("[PT] Passthrough attivo. Logging sospeso. Scrivi +++ per uscire.");
  if (fileOpen) logFile.flush();
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
      case 'f': if (fileOpen) { logFile.flush(); DBGLN("[CMD] Flush eseguito"); } break;
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

#if FILE_TIMESTAMP
  SdFile::dateTimeCallback(fatDateTime);
#endif
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
        if (ok) { st.sentences++; parseNmea(lineBuf); }
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
  tryOpenLogFile();
  if (fileOpen && now - lastFlush >= FLUSH_INTERVAL) {
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

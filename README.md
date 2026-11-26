# XIAO GPS Logger

Logger GPS per aeromodelli basato su **Seeed XIAO nRF52840**, ricevitore GPS **BN-220** (u-blox M8) e scheda **microSD**.
Registra le frasi NMEA fino a **10 posizioni al secondo** su file di testo, con diagnostica completa via USB.

## Caratteristiche

- Registrazione NMEA (RMC + GNS + VTG) su microSD, un nuovo file a ogni accensione,
  nominato con **data e ora locali** del GPS: `20261126/202103.TXT`
- File leggibili direttamente da [RZ Flight Dashboard](https://github.com/rzannori/rz-flight-dashboard)
- Configurazione automatica del GPS all'avvio tramite comandi UBX:
  - frequenza 1 / 5 / **10 Hz**
  - seriale a 38400 baud
  - modello dinamico *Airborne* (adatto ad aeromodelli e acrobazia)
- Salvataggio fisico su SD ogni secondo (configurabile): in caso di distacco dell'alimentazione si perde al massimo ~1 s
- Scarta le frasi con checksum errato
- Riga proprietaria `$PXIAO` ogni secondo con temperatura della CPU (e accelerometro/giroscopio sulla versione *Sense*)
- Recupero automatico della SD in caso di errore
- LED di stato e report diagnostico sulla seriale USB
- Radio Bluetooth spenta: nessuna interferenza con i radiocomandi a 2,4 GHz

## Hardware

| Componente | Note |
|---|---|
| Seeed XIAO nRF52840 (o nRF52840 Sense) | |
| GPS BN-220 | u-blox M8, alimentazione 3,0–5,5 V |
| Modulo microSD (SPI) | adattatore semplice → 3,3 V; modulo con regolatore AMS1117 → 5 V |
| Regolatore step-down 5 V | ingresso almeno fino a 10 V (es. Pololu D24V5F5, D36V6F5, MP1584 tarato a 5,0 V) |
| Diodo Schottky (SS14 / 1N5817) | facoltativo, per poter collegare l'USB con la ricevente accesa |

### Collegamenti

```
BN-220  TX  -> D7  (RX di Serial1)
BN-220  RX  -> D6  (TX di Serial1)
BN-220  VCC -> 3V3      GND -> GND

SD      SCK -> D8
SD      MISO-> D9
SD      MOSI-> D10
SD      CS  -> D2   (modificabile con SD_CS_PIN)
SD      VCC -> 3V3 o 5V (vedi tabella)   GND -> GND
```

### Alimentazione dalla ricevente (Jeti, 6 V o 8,4 V)

La tensione della barra servi (6 V con BEC, fino a 8,4 V con 2S LiPo/Li-Ion o BEC HV) è **troppo alta** per la XIAO:
il pin 5V (che è collegato direttamente alla presa USB) regge al massimo circa 5,5 V.
Serve sempre un regolatore.

```
Cavo servo da un canale libero della RX
  marrone/nero (-) ───────────────► GND regolatore ── GND XIAO
  rosso        (+) ───────────────► IN  regolatore
                                     OUT 5 V ──►|── pin 5V XIAO
                                              diodo Schottky (facoltativo)
```

- Non usare la porta **Ext** della ricevente (è per telemetria/EX Bus).
- Misura l'uscita del regolatore con il tester **prima** di collegare la XIAO.
- Il consumo totale è di circa 60–80 mA (picchi ~150 mA durante le scritture su SD).

#### Batteria di riserva (facoltativa)

Sui pad **BAT+ / BAT−** sul retro della XIAO si può saldare una piccola LiPo 1S (150–300 mAh, con circuito di protezione).
Viene ricaricata dal pin 5V e, se l'alimentazione della ricevente manca, la XIAO prosegue a batteria senza interruzioni.
In questo caso GPS e SD vanno alimentati dal pin **3V3**, perché a batteria il pin 5V va a zero.

## Installazione del software

1. **Arduino IDE** → *File → Preferenze → URL aggiuntivi per il Gestore schede*:
   ```
   https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
   ```
2. *Gestore schede* → installa **Seeed nRF52 Boards** (**non** la versione *mbed-enabled*).
3. Scheda: **Seeed nRF52 Boards → Seeed XIAO nRF52840** (o *Sense*).
4. Apri `gps_logger/gps_logger.ino`, compila e carica.
   Per entrare in modalità bootloader premi due volte velocemente il tasto reset.

Librerie: `SD` e `SPI` sono incluse nell'IDE. Solo se usi l'IMU della versione Sense serve **Seeed Arduino LSM6DS3**.

## Configurazione

Tutti i parametri sono in cima allo sketch:

| Parametro | Default | Descrizione |
|---|---|---|
| `SD_CS_PIN` | `D2` | Pin CS della microSD |
| `GPS_CONFIGURE` | `1` | Configura il GPS all'avvio |
| `GPS_RATE_HZ` | `10` | Posizioni al secondo (1, 5, 10) |
| `GPS_BAUD_FAST` | `38400` | Baud del GPS dopo la configurazione |
| `GPS_DYN_MODEL` | `8` | 0 = portatile, 7 = aereo <2g, 8 = aereo <4g |
| `GPS_GSA_GSV` | `0` | 1 = aggiunge il dettaglio satelliti (1 volta al secondo) |
| `TZ_OFFSET_H` | `1` | Fuso orario rispetto a UTC per i nomi dei file (Italia = 1) |
| `TZ_EU_DST` | `1` | Applica l'ora legale europea |
| `NODATE_TIMEOUT_S` | `120` | Se il GPS non fornisce la data entro questo tempo, usa i nomi `GPS000.TXT`... |
| `FILE_TIMESTAMP` | `1` | Data/ora del file sulla SD (visibile in Esplora risorse); se non compila metti `0` |
| `FLUSH_INTERVAL` | `1000` | ms tra un salvataggio fisico su SD e l'altro |
| `LOG_ONLY_VALID` | `1` | Scrive solo frasi con checksum corretto |
| `LOG_EXTRA` | `1` | Riga `$PXIAO` con temperatura CPU (+ IMU) |
| `USE_IMU` | `0` | 1 = registra accelerometro/giroscopio (solo *Sense*) |
| `DEBUG` | `1` | 0 = nessun messaggio su USB |

## LED di stato

| LED | Significato |
|---|---|
| Verde lampeggiante | GPS con fix valido |
| Blu lampeggiante | GPS riceve, ma senza fix |
| Blu lampeggio veloce all'avvio | attesa del Serial Monitor (max 3 s) |
| Rosso fisso | errore SD |
| Rosso lampeggiante | nessun dato dal GPS |

## Debug via USB

Serial Monitor a **115200 baud**. Ogni 5 s viene stampato un report:

```
---------------- STATO ----------------
Uptime         : 125 s
GPS            : riceve dati  (1900 byte/s, ultimo dato 40 ms fa)
Frequenza RMC  : 10.0 Hz (attesa 10 Hz), seriale GPS 38400 baud
Fix / satelliti: SI / 9   HDOP 0.92   ora UTC 14:32:07
Frasi NMEA     : 2500 valide, 0 checksum errati, 0 troppo lunghe
SD             : OK  file 20261126/143207.TXT  (2625 righe, 172310 byte)
Errori SD      : 0 scritture fallite, 0 reinizializzazioni, flush max 13 ms
Temperatura CPU: 27.5 C
---------------------------------------
```

Comandi (da scrivere nel Serial Monitor):

| Tasto | Funzione |
|---|---|
| `h` | Elenco comandi |
| `s` | Stato immediato |
| `e` | Eco delle frasi NMEA on/off |
| `r` | Report automatico on/off |
| `f` | Flush immediato su SD |
| `l` | Elenco cartelle e file sulla SD |
| `g` | Riconfigura il GPS |
| `p` | Passthrough USB ↔ GPS (per u-center), uscita con `+++` |

## Formato dei file

### Nomi

Una cartella per giorno e un file per ogni accensione, con data e ora **locali** (fuso e ora legale da `TZ_OFFSET_H` / `TZ_EU_DST`):

```
20261126/
    143207.TXT      ← 26/11/2026, registrazione iniziata alle 14:32:07
    161540.TXT
20261127/
    101205.TXT
```

- La libreria SD accetta solo nomi corti (8 caratteri + estensione): per questo la data è nella cartella e l'ora nel file.
- Il file viene creato appena il GPS conosce la data, di solito pochi secondi dopo l'accensione e spesso prima del fix.
  Le righe ricevute prima, senza data né posizione, non vengono registrate.
- Se la data non arriva entro `NODATE_TIMEOUT_S` (120 s), il logger registra comunque su `GPS000.TXT`, `GPS001.TXT`... nella cartella principale.
- Anche la data/ora del file sulla SD è quella del GPS (con `FILE_TIMESTAMP 1`).

### Contenuto

Testo NMEA standard, una frase per riga:

```
$GNRMC,143207.00,A,4507.12345,N,00740.12345,E,13.66,182.4,231126,,,A*xx
$GNGNS,143207.00,4507.12345,N,00740.12345,E,AAN,09,0.92,312.4,47.1,,,V*xx
$GNVTG,182.4,T,,M,13.66,N,25.30,K,A*xx
$PXIAO,125034,27.5*xx
```

| Frase | Contenuto |
|---|---|
| `RMC` | ora, data, posizione, validità del fix (`A` = valido, `V` = non valido) |
| `GNS` | posizione, quota, satelliti, HDOP |
| `VTG` | velocità (km/h) e rotta |

`$PXIAO,<millis>,<temperatura CPU>[,ax,ay,az,gx,gy,gz]` è una frase proprietaria: i programmi NMEA la ignorano.

### Lettura con RZ Flight Dashboard

La combinazione RMC + GNS + VTG è quella che la dashboard riconosce (la stessa dell'SM GPS): basta aprire il file
`GPSxxx.TXT` con **Apri → Dal computer / telefono**. Il formato viene riconosciuto dal contenuto, l'estensione non conta.

Canali disponibili: posizione, velocità, rotta, quota GPS, quota relativa, satelliti, HDOP.
Il BN-220 non ha barometro: la quota è quella GPS (meno precisa di quella barometrica) e il canale *Vario* non è presente,
quindi l'analisi di termiche e planata è limitata.

> Non tornare a GGA: la dashboard legge la quota **solo** dalle frasi GNS e scarta i file senza quota.

Gli stessi file si possono aprire anche con **GPS Visualizer** (gpsvisualizer.com), **GPSBabel** (conversione in GPX/KML per Google Earth), QGIS e simili.

## Risoluzione dei problemi

| Problema | Soluzione |
|---|---|
| Errore `uses VFP register arguments ... libc_nano.a does not` | Rimuovi `Documents\Arduino\libraries\Adafruit_TinyUSB_Library` (va usata quella interna al core), cancella la cache `%LOCALAPPDATA%\Temp\arduino` e ricompila |
| `Multiple libraries were found for "SD.h"` | Togli la copia in `Documents\Arduino\libraries\SD` e usa quella integrata |
| `Python non è stato trovato ... exit status 9009` | Le versioni recenti del core richiedono Python: installalo da python.org (o attiva `python.exe` in *Impostazioni → App → Impostazioni app avanzate → Alias di esecuzione app* se usi il *Python install manager*) e riavvia l'IDE |
| `SD non trovata` | Controlla cablaggio, `SD_CS_PIN`, tensione del modulo (3,3 V o 5 V) e formattazione FAT32 |
| `Nessun byte dal GPS` | Inverti TX/RX del BN-220 (D6/D7) e controlla l'alimentazione |
| Frequenza RMC più bassa del previsto | Prova il comando `g`; se persiste usa `GPS_RATE_HZ 5` |
| Molti checksum errati | Disturbi o dati persi: scendi a 5 Hz, tieni i fili corti e lontani da ESC e cavi di potenza |
| `SD: in attesa della data GPS` | Normale nei primi secondi: il file viene creato appena il GPS ha data e ora |
| Errore di compilazione su `SdFile::dateTimeCallback` | Metti `FILE_TIMESTAMP 0`: i nomi con la data funzionano lo stesso |
| Nessun fix | Il primo aggancio richiede da 30 s a qualche minuto **all'aperto** |

# Datenlogger mit Web-Server für Viessmann Trimatik-MC
Zur Überwachung der Arbeitsweise der Heizungsregelung "Trimatik-MC" werden die Zustände von Ein- und Ausgängen aus der Kommunikation zwischen Regelbox und Schaltuhr aufgezeichnet. Der Gasverbrauch wird durch einen Reed-Kontakt am Gaszähler erfasst. Der Logger verbindet sich mit einem WLAN. Über sein Web-Interface http://trimatik können die Daten grafisch dargestellt werden. Die Aufzeichnung kann als CSV oder JSON heruntergeladen werden.

## Beispiel
![Beispielanzeige des Web-Interfaces](/images/WebIF.png)

Auf dem Web-Interface sind dargestellt:
- Temperaturverläufe von Kesselwasser, Warmwasser und Außentemperatur
- Schaltzustände der Pumpen, des Brenners und der Schaltuhr
- mittlere Heizleistung, berechnet aus dem Gasverbrauch über ein Brennerintervall
- Soll-Vorlauftemperatur, berechnet aus Programm, Außentemperatur und den eingestellten Kennlinienparametern.
Aus dem Heizkennliniendiagramm der Trimatik-Dokumentation habe ich diese Näherung abgeleitet:
  $$Vorlauftemperatur = \left( RT + Niveau \right) - Neigung \cdot \left(  1,7 \cdot \left(  AT-RT \right) + 0,018 \cdot \left( AT-RT \right) ^2 \right) $$
  * RT = Soll-Raumtemperatur: Normalbetrieb = 20°C+Tagkorrektur (Sonne); reduzierter Betrieb = 14°C+Nachtkorrektur (Mond)
  * AT = Außentemperatur
- aktuelles Datenpaket der Regelung

![Lengende zum Web-Interface](/images/WebIF_Legende.png)

### Schnellstart / Verwendung
1. Die Dateien "TrimatikLogger.ino", "html.h", "logger.h", "wifikey-vorlage.h" und "example.h" in das Verzeichnis "TrimatikLogger" herunterladen
2. Die Zugangsdaten zum WLAN müssen bereitgestellt werden: dazu in der Vorlage "wifikey-vorlage.h" die Zugangsdaten eintragen und als "wifikey.h" speichern. "wifikey.h" wird nicht durch git synchronisiert (.gitignore).
3. Konnte sich der Logger am WLAN anmelden, ist das Web-Interface über "http://trimatik" erreichbar
4. Für den echten Betrieb:
   - Hardwareanbindung an die Trimatik herstellen (siehe [Hardware](#hardware))
   - in "TrimatikLogger.ino" die Zeile "#define BEISPIEL" auskommentieren um die Beispieldaten zu entfernen

## Trimatik-Kompatibilität
Entworfen ist das Projekt für die Viessmann Trimatik-MC von 1/1995, die noch keine der im OpenV-Wiki beschriebenen Datenanbindungen hat. Sie besteht aus:
- Trimatik-MC 7450263
- Regelbox 7408217
- Schaltuhr 9519249

Die dabei genutzte interne Datenverbindung ist nur in den neueren Regelbox/Schaltuhr-Kombinationen vorhanden:
- Regelbox: 7408217, 7408421, 7814549
- Schaltuhr 9519249 (mit Fragezeichentaste)

Ältere Trimatik-MC (7410065, 7410065-A, 7410260, 7410261, 7410261-A) und Miromatic-MC können mit den neueren Elektroniken aufgerüstet werden.
Die Trimatik-MC/B 7450365-B ist anders aufgebaut, bei ihr dürfte der Datenzugang über den Stecker 58/24+25 ("2-Draht-Bus") möglich sein. Olli Z. hat unter [Viessmann Eurola OC mit 2-Drahtbus nachrüsten](https://www.mikrocontroller.net/topic/543681) die Bedieneinheit Comfortrol 7450180 untersucht, die auch für die 7450365-B geeignet ist.
Die Trimatik-MC verfügt ebenfalls über digitale Kommunikation am Stecker 58/24+25, jedoch mit anderem Protokoll. Spätere Geräte verfügen über den Stecker 145 mit KM-Bus.

## Datenabgriff
Regelbox und Schaltuhr tauschen über die zwei Kontakte 3 ('A') und 6 ('B') Daten aus.

Platziert man den Mikrokontroller in der Schaltuhr, können die Daten direkt dort drin abgegriffen werden.
Ich habe die Datenleitungen aus der Trimatik nach außen auf einen Stecker herausgeführt, an den der Logger angeschlossen wird.
Je nach Version der Regelbox kann der Abgriff von A/B unterschiedlich sein. Bei meiner 7408217 sind neben dem 9ten DIP-Schalter von S1 ein weiteres Paar Lötaugen vorhanden, wie für einen 10ten DIP-Schalter. Das obere Lötauge ist 'B', das untere 'A'. Bei anderen Ausführungen der Regelbox ist ein 10fach DIP-Schalter bestückt, bei dem der 10te DIP-Schalter auch A und B ist.

Ich habe Stiftkontakte in die Lötaugen bestückt und Leitungen zum Datenabgriff von der Hauptplatine aus von hinten in die Einbauöffnung der Regelbox geführt. Die Regelbox wird an den Leitungen vorbei eingesetzt und die Leitungen danach aufgesteckt.
![Regelbox mit A/B-Stiften](/images/Regelbox_AB.jpg)
![Einbauöffnung mit A/B-Leitungen](/images/AB-Leitungen.jpg)
![Regelbox mit angeschlossenen A/B-Leitungen](/images/Regelbox_AB-Abgriff.jpg)

## Datenformat auf A/B
Auf 'B' sendet die Regelbox Anfragen und eigene Zustandsdaten, auf 'A' sendet die Schaltuhr die angefragten Daten. 

Ca. alle 1,7s findet ein Datenaustausch statt. Die seriellen Daten werden in 8 Bit, 1 Stopbit, ungerade Parität, 1200 Bit/s übertragen (5V-Pegel).

Jeder Datenaustausch wird durch mind. 22ms Low und 1,66ms High (2 Bitzeiten) auf 'A' eingeleitet, was als Break-Condition interpretiert werden kann. Danach folgen die 15 Datenbytes eines Telegramms und eine Pause von 1,47s.
![Beginn eines Telegramms](/images/AB-Telegramm.png)
- Die Regelbox sendet zwei Frames in einem Abstand von ca. 2 Bitzeiten. 
- Die Schaltuhr antwortet nach ca. 0,3 Bitzeiten.
- Die Regelbox macht zwischen den Byte-Tripel eine Pause von ca. 15 Bitzeiten.
- Die Regelbox beginnt alle ca. 1,695s diesen Datenaustausch.
- Nach 13 Telegrammen wird der Break auf die Dauer eines Telegrammintervalls ausgedehnt und währenddessen auf dem Zweidrahtbus eine Nachricht gesendet.

Die Daten des Telegramms interpretiere ich folgendermaßen:
|Kanal B| A | Bedeutung |
|-----|--|-----------|
|0F 7F|  | Abfrage "Datumswechsel" |
|     |84| Antwort der Schaltuhr: zwischen 0:00 und 0:01 0xA4, sonst 0x84 |
|0F 45|	 | Abfrage "aktives Schaltkanäle" |
|     |00| Antwort der Schaltuhr: 0x11 TagHK1, 0x22 TagHK2, 0x44 WW, 0x88 Zirkulation |
|1F 2F 00| | Status der Regelung: 0=fehlerfrei, sonst 3F 2F xy |
|1F 3A 3C| | Anzeigewert; normalerweise Kesseltemperatur |
|1F 31 10| | wechselnder Inhalt: z.B. Außentemperatur |

Das letzte Datentripel wechselt zwischen folgenden Parametern:
|Kanal B| Bedeutung |
|-------|-----------|
|1F 31 xx|	Außentemperatur; xx = 4&middot;Außentemperatur [°C], vorzeichenbehaftet |
|1F 32 xx|	Vorlauftemperatur; xx = 2&middot;Vorlauftemperatur [°C] |
|1F 33 xx|	Kesseltemperatur; xx = 2&middot;Kesseltemperatur [°C] |
|1F 34 FF|	Bedeutung unbekannt, evtl. digit. Raumtemperatursensor; bei mir immer 0xFF |
|1F 35 xx|	Warmwasserspeichertemperatur; xx = 2&middot;Wassertemperatur [°C] |
|1F 3A xx|	Anzeigewert; xx = 2&middot;Kesseltemperatur [°C] |
|1F 25 xx|	Relaiszustände; bitcodiert: 80=HKB, 40=HKA, 20=MischerAuf, 10=MischerZu, 08=WW-Zirkul, 04=WW-Pumpe, 01=Brenner |
|1F 26 13|	Bedeutung unbekannt; bei mir immer 0x13 |

Nach jeweils 13 dieser Datensequenzen wird der einleitende Low-Pegel von 22ms auf 1,72s verlängert. In diesem Zeitraum wird auf Stecker 58 ein Datentelegramm gesendet.

## Stromversorgung für den Trimatik-Logger
In der Schaltuhr steht nur eine durch den Vorwiderstand R402 in der Regelbox begrenzte Spannung zur Verfügung. Wird dieser Vorwiderstand in der Regelbox überbrückt, stehen die 12V des darin befindlichen 7812T zur Verfügung, und können dann mittels Schaltregler den Logger versorgen.

Soll die Spannungsversorgung nach außen geführt werden, gibt es auf der Rückseite der Hauptplatine einige Abgriffpunkte für 5V, oder auch ungeregelte 30V für einen Schaltregler.

![Versorgungsspannungen an der Hauptplatine](/images/Hauptplatine_5V.jpg)

Ich habe den 5V-Abgriff direkt an den Lötungen des Steckverbinders zur Regelbox angelötet und nach außen geführt.

## Zählimpulse des Gaszählers
Der Gaszähler "Honeywell BK-G4MT" hat an der letzten Stelle des Zählwerks einen Magneten, mit dem die Hundertstel m³ gezählt werden können. In der Mulde unterhalb des Zählwerks kann ein entsprechender Reed-Kontakt befestigt werden. Vorgesehen sind z.B. Typ IN-Z61 oder IN-Z62. Ich habe einen Standard-Reed-Kontakt (Littlefuse 59140) bearbeitet und in die Mulde geklemmt. Der Reedkontakt ist bei Zählerständen x,xx1 m³ bis x,xx9 m³ geöffnet.

## Hardware
Entworfen ist der Trimatik-Logger für den "Wemos S2 mini" mit dem ESP32-S2 und ESP32. Die Beschreibung hier bezieht sich auf den "Wemos S2 mini".

Die seriellen 5V-Datensignale A und B werden über Schottky-Dioden und interne Pullups über die UARTs gelesen.
Der ursprüngliche Aufbau nutzt beide UARTs, um die Daten auf A und B unabhängig voneinander zu erfassen. Da A und B aber nie gleichzeitig Daten übertragen, können beide über Wired-AND auf eine einzige UART geführt werden.
- 'A' ist über eine 1N5817 an GPIO16 angeschlossen und an 'U0RXD' mit internem Pull-Up geführt.
- 'B' ist über eine 1N5817 an GPIO18 angeschlossen und an 'U1RXD' mit internem Pull-Up geführt.

Der Gas-Zählimpuls wird über eine 1N5817 an einen 1k-Pull-Up und eine weitere 1N5817 gegen Masse an GPIO21 geführt.

Beim "Wemos S2 mini" sind weiterhin diese Ein-/Ausgänge belegt:
- GPIO0 ist der seitlich Taster an der Platine (herausstehend)
- GPIO15 ist die blaue LED auf der Platine
- GPIO19 ist USB D-
- GPIO20 ist USB D+

![einfache Verdrahtung des Wemos S2 mini, Oberseite](/images/Wemos_oben.jpg)
![einfache Verdrahtung des Wemos S2 mini, Unterseite](/images/Wemos_unten.jpg)

## Software
Entwickelt ist der Trimatik-Logger auf Arduino 1.8.12 mit der ESP32-Bibliothek v2.0.9 (ESP-IDF v4.4.4) für
- "Wemos S2 mini" mit ESP32-S2: Arduino-IDE-Board "LOLIN S2 MINI"
- "ESP32 Dev Kit" oder "ESP32-CAM" mit ESP32: Arduino-IDE-Board "ESP32 Dev Module"

Auf meinem "Wemos S2 mini" befindet sich ein "ESP32-S2F H4" ohne PSRAM. Ist zusätzlich PSRAM vorhanden, kann dieser als größerer Logging-Speicher oder für weitere Funktionen eingesetzt werden, z.B. EMail-Versand. PSRAM ist z.B. auf "ESP32-CAM" oder "Wemos S2 mini" mit "ESP32-S2F N4R2" verfügbar.

### UART
UART0 und UART1 werden mit 1200baud 8E1 betrieben.
Die RX-Interrupts legen die Daten in einen gemeinsamen Puffer ab. Solange die Interrupts aktiviert sind, ist die zeitliche Reihenfolge im Puffer korrekt. Während eines OTA-Updates sind die Interrupts jedoch gesperrt und die Reihenfolge wird zerstört. Dies kann durch Wired-AND der Datenleitungen 'A' und 'B' zusammen auf eine UART umgangen werden.
Werden nach 1s keine weiteren Daten empfangen, wird das Datenpaket ausgewertet.

### Datenpuffer
Der Ringpuffer, in dem die Aufzeichnung gespeichert wird, besteht aus einem Start-Werte-Satz, bis zu 13200 Einträgen und einem Ist-Werte-Satz. Jeder Eintrag besteht aus Zeitdifferenz zum vorherigen Eintrag, Eintragstyp und Wert.
Neu eingehende Daten werden mit dem Ist-Werte-Satz verglichen und ggfs. in einem neuen Eintrag abgelegt.
Ist der Ringpuffer vollständig gefüllt, wird der älteste Eintrag in den Start-Werte-Satz übernommen und sein Platz wird frei für den neusten Eintrag.
Im Heizbetrieb lassen sich so knapp 2 Tage, im Sommerbetrieb ca. 2 Wochen Aufzeichnung vorhalten.

### Gaszähler
Der Gas-Zählimpuls ist software-entprellt.

### Web-Interface
Das Web-Interface bietet:
- Darstellung und Aufschlüsselung des aktuellen Datenpakets der Trimatik, Gaszählerstands und Pufferzustands (WebSockets)
- Temperatur-, Leistungs- und Status-Diagramm mit Zoom-/Scroll-Funktion (Binärdaten-Download)
- Einstellregler der Heizkennlinie zur Berechnung der Soll-Vorlauftemperatur
- Download der Messwerte als CSV mit optionaler Angabe des Startzeitpunkts "/messwerte.csv?since=xxx"
- Abfrage der Messwerte als JSON mit optionaler Angabe des Startzeitpunkts "/messwerte.json?since=xxx"
- Abfrage der Messwerte als Binärdaten mit optionaler Angabe des Startzeitpunkts "/messwerte.bin?since=xxx"
- OTA-Update

### Weitere Features
- Reset- und Update-fester Speicher der Logger-Daten mit Reset-Zähler
- Uhrzeit NTP-synchronisiert mit Rückwärtskorrektur bei verspäteter NTP-Verfügbarkeit
- CORS ist deaktiviert, um eine einfache Entwicklung von lokalen Web-Interfaces zu ermöglichen
- WiFi-Mode kann geändert werden (g/n)
- WiFi-Scan "/wifiscan.txt"
- Kennlinien-Parameter-Abfrage "/param.json"
- Log-Report "/logreport.txt" - Konsistenzprüfung der Daten
- WebSocket-Funktionen:
  * Setzen der Parameter der Heizkennlinie und Soll-Warmwassertemperatur
  * Setzen des Gaszählerstands
  * Setzen des WiFi-Standards 'g' oder 'n'
  * Anpassen der Ringpuffergröße

### Deaktivierte Funktionen:
- DYNAMIC_LOGMEM: Der Ringpuffer wird nicht statisch sondern dynamisch allokiert - wird für PSRAM benötigt
- USE_PSRAM: allokiert den Ringpuffer im PSRAM falls vorhanden
- LOGALL: Vorlauftemperatur, 0x26 und 0x34 werden ebenfalls aufgezeichnet und per CSV geliefert

## Anmerkungen
Die Kennlinienparameter dienen ausschließlich der Berechnung der Soll-Vorlauftemperatur. Sie werden nicht von der Trimatik gelesen und es ist nicht möglich die Regelung damit zu beeinflussen. Insofern hat der Einstellregler für die Wassertemperatur keine Funktion, außer einer Merkhilfe, welche die Einstellung die Trimatik hat.

In der Statusnachricht über WebSocket werden alle bekannten Werte übermittelt. Im Puffer werden nur die im WebInterface verwendeteten Werte abgelegt. Um alle Werte aufzuzeichnen kann LOGALL verwendet werden.

Damit die Daten ein Firmware-Update überleben, muss die Adresse des Speicherblocks 'logBuf' in beiden Firmwares gleich sein. Die Adresse der aktuellen Firmware lässt sich über http://trimatik/info.txt abfragen, die der neuen Firmware steht in dem MAP-File im %TEMP%\arduino-cache

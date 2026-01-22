/*
  Logger für die B-/A-Signale der Trimatik-MC Schaltuhr
  Arduino 1.8.12 mit ESP32 v2.0.9 (ESP-IDF v4.4.4)

  für Wemos S2 mini, Arduino-Board "LOLIN S2 MINI"
  ESP32-S2FH32 : 240MHz 32Bit-Single-Core mit 4MB Flash, 320kB RAM, 2MB PSRAM, WiFi, ULP-Koprozessor
  ChanB: Serial1 ist RX=IO18, TX=IO17 (UART1-Std.: RX=IO18, TX=IO17)
  ChanA: Serial0 ist RX=IO16, TX=IO13 (UART0-Std.: RX=IO44, TX=IO43)

  Alternativ auf ESP32 (z.B. ESP32-CAM)
  ESP32 : 340MHz 32Bit-Dual-Core mit 4MB Flash, 520kB RAM (320 KB of DRAM and 200 KB of IRAM)
  ChanB: "Serial1" ist RX=IO14, TX=IO15 (UART1-Std.: RX=IO9/TX=IO10 durch PSRAM belegt)
  ChanA: "Serial2" ist RX=IO2, TX=IO13 (UART2-Std.: RX=IO16/TX=IO17 durch PSRAM belegt)
  Debug: "Serial" ist RX=IO3, TX=IO1
  - IO4 ist die Flash-LED, leuchtet bei '1'

  Anbindung Wemos an die Trimatik-MC bzw. Gaszähler
  - GPIO18 : 'B' mit 1200 8E1 : 'U1RXD'
  - GPIO16 : 'A', evtl. 1200 8E1 : 'U0RXD'
  - GPIO21 : Gaszähler, LOW von 0,009 bis 0,011

  Weiterhin sind diese Ein-/Ausgänge beim Wemos vorhanden:
  - GPIO0 ist der Taster am Gehäuse (herausstehend)
  - GPIO15 ist die blaue LED im Gehäuse
  - GPIO19 ist USB D-
  - GPIO20 ist USB D+

  Auf 'B' werden 0x0F 0x7F und 0x0F 0x45 gesendet, das mit einem Byte auf 'A' beantwortet wird,
  Danach werden 0x1F aa bb auf 'B' gesendet.
  Es wird mit 1200 baud, 8E1 übertragen.
  Vor jedem Telegramm wird auf B ein UART_BREAK (0-Pegel) von 22,07ms oder 1,717s ausgeführt. Dieser kann zum Zurücksetzen des Empfangspuffers verwendet werden.
  Danach folgt für zwei Bitperioden der 1-Pegel, bevor das 0x1F gesendet wird.
  Bei Fehlern wird 0x3F 2F XY gesendet (statt 1F 2F 00): für Display !X:Y

  Beispiel:
  A:       84       00                            (von Schaltuhr)
  B: 0f 7f    0f 45    1f 2f 00 1f 3a 3f 1f 31 12 (von Steuerung)

  0f 7f = 84/a4 von Schaltuhr, von 0:00-0:01 'a4' (1,7s)
  0f 45 = 00/11/22/44/88 von Schaltuhr, je nach aktivem Kanal (1,7s)
  1f 25 = Bitmaske Ausgänge (11,9s)
  1f 26 13 -unbekannt- (11,9s)
  1f 2f 00 Status ok (1,7s)
  3f 2f xy Fehler x:y (1,7s)
  1f 31 = AT*2 (11,9s)
  1f 32 = VT*2 (11,9s)
  1f 33 = KT*2 (11,9s)
  1f 34 FF -unbekannt- (11,9s)
  1f 35 = WW*2 (11,9s)
  1f 3a = KT*2/Display Normalanzeige (1,7s)

  Datenpuffer für:
  - Gas
  - 1f 31 Außentemp
  - 1f 3a Kesseltemp
  - 1f 35 Warmwasser
  - 1f 25 Ausgänge
  - 0f 45 Schaltuhr Programm
  - 0f 7f Schaltuhr Tag
  - 1f 2f / 3f 2f Status

  Aufbau Datenpuffer:
  - Datenblock für alle Start-Werte: 4 Byte Zeit, 4 Byte Gas + je 1 Byte AT, KT, WW, Ausgänge, Programm, Tag, Status, ggfs. '26', '32', '34' und '35'
  - Datenblock für alle Jetzt-Werte: 4 Byte Zeit, 4 Byte Gas + je 1 Byte AT, KT, WW, Ausgänge, Programm, Tag, Status, ggfs. '26', '32', '34' und '35'
  - Ringpuffer: Byte 0+1: Zeitinkrement, Byte 2: Wert-ID, Byte 3: Wert; bei Gas ist es das Inkrement

  Die Daten im Ringpuffer können gekürzt werden. Damit ist es möglich, den Puffer für eine Firmware vorzubereiten, die eine andere Puffergröße verwendet.
  Die Daten werden gekürzt und dabei umsortiert, sodass sie wieder bei Index 0 beginnen. Damit kann auf einen kleineren wie größeren Puffer umgebaut werden.
  Die WebSocket-Msg lautet "LogLen:####"

  10.11.2024 Hinweis: time() überlebt Warmstarts.
             In setup() wird startTS = time()-millis()/1000 gesetzt, sodass (solange time() ungültig ist) startTS den zu millis() relativen Zeitpunkt des Kaltstarts anzeigt.
             Der absolute Kaltstartzeitpunkt ist also time() - startTS - millis()/1000
             Die TS-Korrektur wurde in die time_is_set() verlegt: TSreal = time() - startTS - millis() + TS
  30.11.2024 Hinweis: während des Updates werden die Trimtik-Daten durcheinandergebracht
             die seriellen Daten werden zwar in den Puffern beider Schnittstellen gesammelt, beim Auslesen geht aber der zeitliche Zusammenhang verloren.
             Abhilfe: Wired-AND von A und B an den Rx

  (Soft-Reset by WDT: 2 Sekunden IO0 drücken, dann beginnt die LED zu blinken, wobei der loop() angehalten ist und der Watchdog den Reset auslöst)
  Soft-Reset: 2 Sekunden IO0 drücken, dabei blinkt die LED. Wenn die 2 Sekunden gedrückt worden sind, wird der Restart() ausgelöst

  mit CDC: logData(57632) = 0x3ffc7b84
  Der Sketch verwendet 851158 Bytes (64%) des Programmspeicherplatzes. Das Maximum sind 1310720 Bytes.
  Globale Variablen verwenden 111600 Bytes (34%) des dynamischen Speichers, 216080 Bytes für lokale Variablen verbleiben. Das Maximum sind 327680 Bytes.

  ohne CDC: logData(57632) = 0x3ffc7914
  Der Sketch verwendet 822714 Bytes (62%) des Programmspeicherplatzes. Das Maximum sind 1310720 Bytes.
  Globale Variablen verwenden 99616 Bytes (30%) des dynamischen Speichers, 228064 Bytes für lokale Variablen verbleiben. Das Maximum sind 327680 Bytes.

  CDC benötigt ca. 15kB internen Speicher. Wenn deaktiviert, geht allderdings die zeitliche Zuordnung der UARTS verloren.

  ESP Mail Client
  -----
  unterstützt PSRAM, wenn in der ESP_Mail_FS.h #define ESP_MAIL_USE_PSRAM aktiv ist.
  Weitere Anpassungen sind darin möglich.

  JavaScript Pinch-Zoom:
  https://konvajs.org/docs/sandbox/Multi-touch_Scale_Stage.html

  in TrimatikLoggerAB.ino.map
  v2.0.14 0x3ffc7144                logBuf
  v2.0.9  0x3ffc7b64  18.11.2023
  v2.0.9 mit esp_get_minimum_free_heap_size() liegt logBuf auf 0x3ffc7b84 (01.12.2024)

  IO0 + Reset, dann
  %LOCALAPPDATA%\Arduino15\packages\esp32\tools\esptool_py\4.5.1/esptool.exe --chip esp32s2 --port COM7 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 %TEMP%\arduino_build_769580/TrimatikLoggerAB2.ino.bootloader.bin 0x8000 %TEMP%\arduino_build_769580/TrimatikLoggerAB2.ino.partitions.bin 0xe000 %LOCALAPPDATA%\Arduino15\packages\esp32\hardware\esp32\2.0.9/tools/partitions/boot_app0.bin 0x10000 %TEMP%\arduino_build_769580/TrimatikLoggerAB2.ino.bin

*/

// Laden von Beispieldaten:
#define BEISPIEL  // der Puffer wird zu Demozwecken mit Beispieldaten gefüllt; für den richtigen Betrieb diese Zeile auskommentieren

// Baudrate der Trimatik-Daten
#define BAUDRATE 1200

//#define DYNAMIC_LOGMEM  // dynamische statt statischer Allokierung des Ringpuffers
//#define USE_PSRAM       // erlaubt die Verwendung von PSRAM, falls vorhanden
//#define FORCE_DRAM        // wenn DYNAMIC_LOGMEM und nicht USE_PSRAM, dann erzwinge MALLOC_CAP_INTERNAL
#define NODEBUG          // keine Debugging-Ausgabe über Serial; CDC belegt Speicher, der für den Ringpuffer benötigt wird, und die UARTs mit 1200 würden bremsen
//#define LOGALL          // mit LOGALL werden mehr Werte gespeichert und übertragen

/*  Zuordnung der UARTs:
 *            ESP32    ESP32-S2  ESP32-S2+CDC
 *  Serial    UART0    UART0     VCI
 *  Serial0                      UART0
 *  Serial1   UART1    UART1     UART1
 *  Serial2   UART2                         
 *  SerialA   Serial2  Serial    Serial0
 *  SerialB   Serial1  Serial1   Serial1
 */
// bei ARDUINO_USB_CDC_ON_BOOT:
// - Serial0 ist UART #0, Serial ist VCI
// ohne ARDUINO_USB_CDC_ON_BOOT:
// - Serial ist UART #0
// immer gibt es Serial1 aus UART #1
#define SerialB Serial1
#if ARDUINO_LOLIN_S2_MINI
  #if ARDUINO_USB_CDC_ON_BOOT
    #define SerialA Serial0
  #else
    #define SerialA Serial
  #endif 
#else
  #define SerialA Serial2
#endif

#ifdef ARDUINO_LOLIN_S2_MINI
  // Portbelegung beim Wemos S2 mini
  #define LED_BUILTIN 15
  #define CHANNEL_A 16
  #define TXDUMMY_A 13  // log_v-Output
  #define CHANNEL_B 18  // U1RXD=18, U1TXD=17
  #define TXDUMMY_B 17
  #define GAS_COUNTER 21
  #define LOGMAX (13200)        // legt die Anzahl Einträge im Ringpuffer fest
#else // ARDUINO_ESP32_DEV
  // Portbelegung beim ESP32-CAM
  #define LED_BUILTIN 33        // leuchtet bei '0'
  #define CAMERA_MODEL_AI_THINKER
  #define LED_FLASH 4           // leuchtet bei '1'
  #define CHANNEL_A 2  // z.B. 2 und 4
  #define TXDUMMY_A 13
  #define CHANNEL_B 14  // z.B. 14 und 15 ; fürs Debugging: U0RXD=3, U0TXD=1
  #define TXDUMMY_B 15
  #define GAS_COUNTER 0
  // bei mehr als 65535 Log-Items muss die Index-Breite angepasst werden!
  #define LOGMAX 14000
#endif

#include "driver/uart.h"
#include <HardwareSerial.h>
//#include "esp32-hal-psram.h"
//#include <math.h>
#include <WiFi.h>         /* %PROGRAMFILES%\Arduino\hardware\arduino\avr\cores\arduino\ */
#include "time.h"
#include "sntp.h"
#include <WiFiClient.h>
//#include <WebServer.h>    /* %LOCALAPPDATA%\Arduino15\packages\esp32\hardware\esp32\1.0.4\libraries\WebServer\src\ */
//#include <Preferences.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#ifdef ESP8266
  #include <Updater.h>
  #include <ESP8266mDNS.h>
  #define U_PART U_FS
#else
  #include <Update.h>
  #include <ESPmDNS.h>
  #define U_PART U_SPIFFS
#endif
#ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
  #if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
    #include "esp32/rom/rtc.h"
  #elif CONFIG_IDF_TARGET_ESP32S2
    #include "esp32s2/rom/rtc.h"
  #elif CONFIG_IDF_TARGET_ESP32C3
    #include "esp32c3/rom/rtc.h"
  #elif CONFIG_IDF_TARGET_ESP32S3
    #include "esp32s3/rom/rtc.h"
  #else
    #error Target CONFIG_IDF_TARGET is not supported
  #endif
#else // ESP32 Before IDF 4.0
  #include "rom/rtc.h"
#endif

#include "hal/gpio_hal.h"
#include "esp_wifi.h"

#include "logger.h"
#include "html.h"
#ifdef BEISPIEL
  #include "example.h"
#endif
#ifdef NODEBUG
  #define DBUG(x)
#else
  #define DBUG(x) x
#endif

#define toggleLED(x) digitalWrite(x,!digitalRead(x))

// Konstanten für die NTP-Konfiguration
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

// Netzwerkdaten WLAN
#include "wifikey.h"
const char* host = "Trimatik";
const char* ssid = mySSID;
const char* password = myWIFIKEY;

// Zeiger auf die Datenstruktur mit Ringpuffer für Aufzeichnung
trimatikLog *logData;
/* in trimaticLog werden diese Werte gespeichert:
  static unsigned char Proogramm = 0;     // 0x45 von Schaltuhr: 0x00/0x11/0x22/0x44 je nach durch das Zeitprogramm aktivem Kanal
  static unsigned char Tag = 0;           // 0x7f von Schaltuhr: 0x84 oder 0xA4
  static unsigned char Relais = 0;        // 0x25 Relais (WW-Pumpe, Brenner, usw.)
  static unsigned char x26 = 0;           // 0x26 unbekannt
  static unsigned char Status = 0;        // 0x2f Status + Reset-Counter
  static unsigned char Kesseltemp1X2 = 0; // 0x3a Displaywert kommt alle 1,7s
  static unsigned char AussentempX2 = 0;  // 0x31
  static unsigned char VorlaufX2 = 0;     // 0x32 Vorlauftemperatur von HK2 mit Mischer
  static unsigned char Kesseltemp2X2 = 0; // 0x33 kommt alle 11,9s
  static unsigned char x34 = 0;           // 0x34 unbekannt
  static unsigned char WarmwasserX2 = 0;  // 0x35 */
// xxx enthält Werte aus A/B, die nicht zugeordnet werden können; xxx wird nicht im Puffer aufgezeichnet
static unsigned char xxx = 0;
// ein Werte-Satz, der in unterschiedlichen Funktionen verwendet wird
static trimatikWerte Werte; // aktueller Werte-Satz einer CSV-/JSON-Zeile (statische Variable), oder in allocLogMem() (einmalig)

// Gaszähler-Entprellung
static unsigned short int gasLvlCnt = 0;  // Level-Timer zur Entprellung

// mutex für den Zugriff auf die zeitrelevanten Variablen
portMUX_TYPE timeMux = portMUX_INITIALIZER_UNLOCKED;
time_t now;                       // Zeitpunkt des loop()-Eintritts, wird für die dt-Berechnung verwendet
time_t startTS;   // enthält den Startzeitpunkt: entweder als Sekunden seit Kaltstart, oder als Uhrzeit des Kaltstarts; wird für die Korrektur anhand millis() benutzt
time_t ntpSyncTS;
bool logRealTS;   // solange noch keine gültige Uhrzeit vom NTP vorliegt, werden die Sekunden seit Kaltstart in ts aufgezeichnet. Dieses Flag kennzeichnet die Aufzeichnung mit gültiger Uhrzeit
// Anzahl der erfolgreichen NTP-Syncs
unsigned short int ntpSyncCount = 0;

volatile bool newdata;     // wird in ISR und SerialCB auf true gesetzt, wenn sich die Daten geändert haben (um zu speichern)

// die Reihenfolge der folgenden resetfesten Blöcke ist wichtig, damit die Anordnung nach einem FW-Update gleich bleibt:
// die Variablen werden im Speicher in der umgekehrten Reihenfolge angelegt, logBuf1 ist zum Padding vorgesehen
// 0x3ffc7ba0 logBuf
// 0x3ffc7b98 Parameter
// 0x3ffc7b84 Ist
// padding logBuf1
#ifndef DYNAMIC_LOGMEM
__NOINIT_ATTR char logBuf[LOGMEMSIZE];  // 0x3ffc7b84
#endif
__NOINIT_ATTR trimatikParameter Parameter;
__NOINIT_ATTR trimatikWerte Ist;
//__NOINIT_ATTR char logBuf1[0x254];

// Puffer für eine serielle Nachricht über A und B; i.d.R. ist sie 16 Bytes lang einschließlich des UART_BREAK-Null-Bytes
#define MSGLEN_MAX 64
unsigned char trimatikMsg[MSGLEN_MAX];
// Anzahl an Bytes im seriellen Empfangspuffer
signed int trimatikMsgLen;
// nur fürs Debugging: Anzahl der Bytes pro Datenpaket, die über die UART empfangen wurde
static unsigned char SerA_cnt = 0;
static unsigned char SerB_cnt = 0;
// der Datenempfang von A/B wird nach einem Timeout beendet:
unsigned long int msgLastByteTS;  // Zeitstempel des zuletzt empfangenen Bytes

// Variablen, die durch WebSocket gesetzt werden können, um Funktionen auszulösen:
// WiFi-Mode umzustellen: der neue Mode wird in WiFi_switch eingetragen und im loop() der WiFi-Mode umgeschaltet
char WiFi_switch = 0; // wenn hier ein Wert gesetzt wird (durch WebSocketMsg), dann wird in der loop() der Modus entsprechend geändert.
// der Ringpuffer kann verkleinert werden, bevor eine FW mit kleinerem Puffer eingespielt wird
// newLogLen wird über WebSocketMsg gesetzt und im loop() verarbeitet
unsigned short int newLogLen = 0;

// Nach einem OTA-Update kann ein Reset im loop() ausgelöst werden
bool restartPending = false;

// Webserver und Websocket des Web-Interfaces
AsyncWebServer httpServer(80);          // Create AsyncWebServer object on port 80
AsyncWebSocket ws("/ws");

static const char *reset_reason(RESET_REASON reason)
{
  switch (reason)
  {
    case 1 : return ("POWERON_RESET");        /**<1,  Vbat power on reset*/
    case 3 : return ("SW_RESET");             /**<3,  Software reset digital core*/
    case 4 : return ("OWDT_RESET");           /**<4,  Legacy watch dog reset digital core*/
    case 5 : return ("DEEPSLEEP_RESET");      /**<5,  Deep Sleep reset digital core*/
    case 6 : return ("SDIO_RESET");           /**<6,  Reset by SLC module, reset digital core*/
    case 7 : return ("TG0WDT_SYS_RESET");     /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8 : return ("TG1WDT_SYS_RESET");     /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9 : return ("RTCWDT_SYS_RESET");     /**<9,  RTC Watch dog Reset digital core*/
    case 10 : return ("INTRUSION_RESET");     /**<10, Instrusion tested to reset CPU*/
    case 11 : return ("TGWDT_CPU_RESET");     /**<11, Time Group reset CPU*/
    case 12 : return ("SW_CPU_RESET");        /**<12, Software reset CPU*/
    case 13 : return ("RTCWDT_CPU_RESET");    /**<13, RTC Watch dog Reset CPU*/
    case 14 : return ("EXT_CPU_RESET");       /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : return ("RTCWDT_BROWN_OUT_RESET"); /**<15, Reset when the vdd voltage is not stable*/
    case 16 : return ("RTCWDT_RTC_RESET");    /**<16, RTC Watch dog reset digital core and rtc module*/
    case 17 : return ("TG1WDT_CPU_RESET");    /**<17, Time Group1 reset CPU*/
    case 18 : return ("SUPER_WDT_RESET");     /**<18, super watchdog reset digital core and rtc module*/
    case 19 : return ("GLITCH_RTC_RESET");    /**<19, glitch reset digital core and rtc module*/
    case 20 : return ("EFUSE_RESET");         /**<20, efuse reset digital core*/
    default : return ("NO_MEAN");
  }
}

static const char *verbose_reset_reason(RESET_REASON reason)
{
  switch (reason)
  {
    case 1  : return ("Vbat power on reset");
    case 3  : return ("Software reset digital core");
    case 4  : return ("Legacy watch dog reset digital core");
    case 5  : return ("Deep Sleep reset digital core");
    case 6  : return ("Reset by SLC module, reset digital core");
    case 7  : return ("Timer Group0 Watch dog reset digital core");
    case 8  : return ("Timer Group1 Watch dog reset digital core");
    case 9  : return ("RTC Watch dog Reset digital core");
    case 10 : return ("Instrusion tested to reset CPU");
    case 11 : return ("Time Group reset CPU");
    case 12 : return ("Software reset CPU");
    case 13 : return ("RTC Watch dog Reset CPU");
    case 14 : return ("for APP CPU, reseted by PRO CPU");
    case 15 : return ("Reset when the vdd voltage is not stable");
    case 16 : return ("RTC Watch dog reset digital core and rtc module");
    case 17 : return ("Time Group1 reset CPU");
    case 18 : return ("super watchdog reset digital core and rtc module");
    case 19 : return ("glitch reset digital core and rtc module");
    case 20 : return ("efuse reset digital core");
    default : return ("NO_MEAN");
  }
}

static const char *translateEncryptionType(wifi_auth_mode_t encryptionType)
{
  switch (encryptionType)
  {
    case (WIFI_AUTH_OPEN):            return "Open";
    case (WIFI_AUTH_WEP):             return "WEP";
    case (WIFI_AUTH_WPA_PSK):         return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):        return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):    return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE): return "WPA2_ENTERPRISE";
    case (WIFI_AUTH_WPA3_PSK):        return "WPA3_PSK";         /**< authenticate mode : WPA3_PSK */
    case (WIFI_AUTH_WPA2_WPA3_PSK):   return "WPA2_WPA3_PSK";    /**< authenticate mode : WPA2_WPA3_PSK */
    case (WIFI_AUTH_WAPI_PSK):        return "WAPI_PSK";         /**< authenticate mode : WAPI_PSK */
    default:                          return "unknown";
  }
}

//*********************** Funktionen für den Log ********************************
// applyLogItem wendet den Ringpuffereintrag mit dem Index 'item' auf den Werte-Satz 'werte' an
void applyLogItem(trimatikWerte *werte, unsigned short int item)
{
  if(!logData) return;
  werte->ts += logData->msg[item].dt;
  switch (logData->msg[item].id)
  {
    case 0x01:
      werte->Gas += logData->msg[item].value;
      break;
    case 0x25:
      werte->Relais = logData->msg[item].value;
      break;
    case 0x26:
      werte->x26 = logData->msg[item].value;
      break;
    case 0x2f:
      werte->Status = logData->msg[item].value;
      break;
    case 0x31:
      werte->Aussen = logData->msg[item].value;
      break;
    case 0x32:
      werte->Vorlauf = logData->msg[item].value;
      break;
    case 0x33:
      werte->Kessel2 = logData->msg[item].value;
      break;
    case 0x34:
      werte->x34 = logData->msg[item].value;
      break;
    case 0x35:
      werte->Wasser = logData->msg[item].value;
      break;
    case 0x3a:
      werte->Kessel = logData->msg[item].value;
      break;
    case 0x45:
      werte->Programm = logData->msg[item].value;
      break;
    case 0x7f:
      werte->Tag = logData->msg[item].value;
      break;
  }
}

// applyLog() wendet den gesamten Ringpuffer auf den Werte-Satz 'werte' an; wird zum Prüfen der Konsistenz mit dem Ist-Satz benutzt
void applyLog(trimatikWerte *werte)
{
  if(!logData) return;
  if (logData->numRecs > 0)
  {
    unsigned short int item;
    // Startindex
    if (logData->numRecs == LOGMAX)
    {
      item = logData->nextRec + 1; // der nextRec ist mit dem Magic belegt, also überspringen
      if (item == LOGMAX) item = 0;
    }
    else
    {
      item = 0;
    }

    while (item != logData->nextRec)
    {
      applyLogItem(werte, item);
      item++;
      if (item == LOGMAX) item = 0;
    } // end while(item != logData->numRecs)
  } // end if(logData->numRecs>0)
}

// initLogData() wird aufgerufen, wenn ungültige logData vorliegen
void initLogData()
{
  memset(&Ist, 0, sizeof(Ist));
  if (logData != 0)
  {
    DBUG(Serial.println("Init logData");)
    memset(logData, 0, sizeof(trimatikLog));
    logData->magic_start = LOGMAGIC_START;
    //logData->numRecs = 0;
    //logData->nextRec = 0;
    logData->gasLvl = digitalRead(GAS_COUNTER);
    if (logData->gasLvl) gasLvlCnt = 50; else gasLvlCnt = 0;
    CAST_ULONG(logData->msg[0]) = LOGMAGIC_END;
    
    //Parameter.resetCount = 0;
    memset(&Parameter, 0, sizeof(Parameter));
    Parameter.Neigung = 12;
    Parameter.TempWasser = 44;

#ifdef TRIMATIK_DATA_H
    // lädt ein Beispiel-Log aus exampleLog, was über /messwerte.bin bezogen werden kann
    // 0: 4 Bytes Werte.ts
    // 4: 4 Bytes Werte.Gas
    // 8: 10x4 Bytes für die Startwerte von Werte.Aussen, Werte.Kessel, Werte.Wasser, Werte.Relais, Werte.Programm, Werte.x26, Werte.Status, Werte.Vorlauf, Werte.x34, Werte.Tag
    // oder
    // 8: 7x4 Bytes für die Startwerte von Werte.Aussen, Werte.Kessel, Werte.Wasser, Werte.Relais, Werte.Programm, Werte.Status, Werte.Tag
    // 48 bzw. 36: numRecs*4 Bytes Records
    #warning Beispieldaten

    unsigned long *start = (unsigned long *)exampleLog;
    logData->Start.ts = start[0];
    logData->Start.Gas = start[1];

    trimatikRecord *rec = (trimatikRecord *)&exampleLog[4+4];
    unsigned long int exampleRecs = (sizeof(exampleLog)-4-4)/sizeof(trimatikRecord);
    //logData->numRecs = 0;
    // übertrage zunächst die Init-Einträge, d.h. alle bei denen dt==0
    while(exampleRecs!=0 && rec->dt == 0)
    {
      logData->msg[logData->numRecs] = *rec;
      logData->numRecs++;
      rec++;
      exampleRecs--;
      if(logData->numRecs == LOGMAX-1) break;
    }
    logData->nextRec = logData->numRecs; // der nextRec zeigt auf den nächsten freien Platz
    // jetzt die Init-Einträge auf den Start-Satz anwenden
    applyLog(&logData->Start);
    // und nun den Rest des Beispiels kopieren
    memcpy(logData->msg, rec, exampleRecs*sizeof(trimatikRecord));
    logData->numRecs = exampleRecs;
    logData->nextRec = logData->numRecs;
    if(logData->nextRec == LOGMAX) logData->nextRec = 0;
    CAST_ULONG(logData->msg[logData->nextRec]) = LOGMAGIC_END;

    Ist = logData->Start;
    applyLog(&Ist);
#endif
  }
}

// bereitet den Log-Speicher vor und prüft auf vorhandene Daten
void allocLogMem()
{
  if (logData == 0)
  {
    #ifdef DYNAMIC_LOGMEM
      // heap_caps_malloc_prefer( LOGMEMSIZE, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT );
      #ifdef USE_PSRAM
        if (psramFound())
        {
          logData = (trimatikLog *)ps_malloc(LOGMEMSIZE);
          DBUG(Serial.printf("ps_malloc(%d) = 0x%08x\n", LOGMEMSIZE, logData);)
        }
      #endif
      #ifdef FORCE_DRAM
        if (logData == 0)
        {
          logData = (trimatikLog *)heap_caps_malloc(LOGMEMSIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          DBUG(Serial.printf("heap_caps_malloc(%d) = 0x%08x\n", LOGMEMSIZE, logData);)
        }
      #else
        if (logData == 0)
        {
          logData = (trimatikLog *)malloc(LOGMEMSIZE);
          DBUG(Serial.printf("malloc(%d) = 0x%08x\n", LOGMEMSIZE, logData);)
        }
      #endif
    #else // wenn nicht DYNAMIC_LOGMEM, also statischer logBuf
      logData = (trimatikLog *)logBuf;
      DBUG(Serial.printf("data(%d) = 0x%08x\n", LOGMEMSIZE, logData);)
    #endif
    if (logData != 0)
    {
      DBUG(Serial.printf("magic_start = 0x%08x (0x%08x), numRecs = 0x%08x, nextRec = 0x%08x, msg[0].dt = 0x%08x\n", logData->magic_start, LOGMAGIC_START, logData->numRecs, logData->nextRec, CAST_ULONG(logData->msg[0].dt));)
      if (logData->magic_start == LOGMAGIC_START)
      {
        unsigned long int item = logData->numRecs;
        DBUG(Serial.printf("Magic-Start found, len=%d", item);)
        item = logData->nextRec;
        if (item < LOGMAX && CAST_ULONG(logData->msg[item].dt) == LOGMAGIC_END)
        {
          DBUG(Serial.printf(", Magic-End found");)
          if (logData->Start.ts != 0) // !(!ts && numRecs) = ts || !numRecs
          {
            // falls Start.ts noch relativ ist (bei Absturz während NTP) muss dieser jetzt korrigiert werden
            if (logRealTS && logData->Start.ts < 1686249016)
            {
              // Start.ts bezieht sich auf die Sekunden seit dem Kaltstart, d.h. der echte Zeitpunkt ist TSreal = now - Kaltstart + TS
              DBUG(Serial.printf(", Start.ts %d=>", logData->Start.ts);)
              logData->Start.ts += startTS;
              DBUG(Serial.print(logData->Start.ts);)
            }

            // Puffer-Start und Ende sind ok, nun die Ist-Werte prüfen
            Werte = logData->Start;
            applyLog(&Werte);

            // mit dem ts etwas toleranter sein: akzeptiert werden Abweichungen von +-2 Sekunden; Werte.ts wird nach Ist.ts übernommen
            time_t dt = Werte.ts - Ist.ts;
            if (-3 < dt && dt < 3) Ist.ts = Werte.ts;
            if (memcmp(&Werte, &Ist, sizeof(Werte)) == 0)
            {
              DBUG(Serial.println(", Data ok");)
            }
            else
            {
              DBUG(
                Serial.println(", Data inconsistent");
                Serial.print("Werte:");
                for (int i = 0; i < sizeof(Werte); i++)
                {
                  Serial.printf(" %02x", ((unsigned char *)&Werte)[i]);
                }
                Serial.print("\nIst:  ");
                for (int i = 0; i < sizeof(Ist); i++)
                {
                  Serial.printf(" %02x", ((unsigned char *)&Ist)[i]);
                }
                Serial.print('\n');
              ) // end DBUG
              initLogData();
            }
          }
          else // if(logData->numRecs!=0 && logData->Start.ts!=0)
          {
            DBUG(Serial.println(", Start.ts unset");)
            initLogData();
          }
        }
        else
        {
          DBUG(Serial.println(", Magic-End NOT found");)
          initLogData();
        }
      }
      else
      {
        DBUG(Serial.println("Magic-Start NOT found");)
        initLogData();
      }
      gasLvlCnt = logData->gasLvl ? 50 : 0;
      DBUG(Serial.printf("magic_start = 0x%08x (0x%08x), numRecs = 0x%08x, nextRec = 0x%08x, msg[%d].dt = 0x%08x, Start.ts = 0x%08x\n",
                         logData->magic_start, LOGMAGIC_START, logData->numRecs, logData->nextRec, logData->nextRec, CAST_ULONG(logData->msg[logData->nextRec].dt), logData->Start.ts);)
    }
    else initLogData();  // falls kein logData vorhanden, dann zumindest den Ist-Wert initialisieren
  }
  // entweder es wurde ein früherer einwandfreier Datenblock gefunden, oder alles wurde auf 0 initialisiert.
}
// logValue übernimmt einen Trimatik-Wert in den Ringpuffer
// src ist ein Byte-Tripel der Trimatik
// src[1]==ID und src[2]==Value werden als neuer Eintrag in den Ringpuffer übernommen
void logValue(const unsigned char *src)
{
  if (logData)
  {
    portENTER_CRITICAL(&timeMux);
    logData->msg[logData->nextRec].dt = now - Ist.ts;
    Ist.ts = now;
    portEXIT_CRITICAL(&timeMux);
    logData->msg[logData->nextRec].id = src[1];
    logData->msg[logData->nextRec].value = src[2];
    logData->nextRec++;
    if (logData->nextRec == LOGMAX) logData->nextRec = 0;
    // die folgenden zwei IFs könnten umgangen werden, wenn statt LOGMAX bei numRecs LOGMAX-1 verwendet wird:
    if (logData->numRecs < LOGMAX) logData->numRecs++;
    if (logData->numRecs == LOGMAX)
    { // der Ringpuffer ist voll: der nächste (älteste) Platz wird überschrieben, d.h. der Start muss vorher diesen Record aufnehmen
      applyLogItem(&logData->Start, logData->nextRec);
    }
    CAST_ULONG(logData->msg[logData->nextRec].dt) = LOGMAGIC_END;
  }
}

// status2str() erzeugt den Info-Text für WebSocket, nachdem ein neues Trimatik-Datenpaket ausgewertet wurde
// der String wird in UTF-8 erzeugt, d.h. '°' wird zu 0xC2B0
// in ANSI / windows-1252 encodierung ist '°' 0xB0
void status2str(char *dst)
{
  dst += sprintf(dst, "\nGas: %.2f m³\nAussen: %.2f °C\nKessel 1: %.1f °C\nKessel 2: %.1f °C\nWarmwasser: %.1f °C", Ist.Gas / 100.0f, (signed char)Ist.Aussen / 4.0f, Ist.Kessel / 2.0f, Ist.Kessel2 / 2.0f, Ist.Wasser / 2.0f);
  dst += sprintf(dst, "\nRely: %02x\n0x26: %02x\nStat: %02x\nVorl: %02x\n0x34: %02x\nProg: %02x\nUhr:  %02x", Ist.Relais, Ist.x26, Ist.Status, Ist.Vorlauf, Ist.x34, Ist.Programm, Ist.Tag);
  if(xxx) dst += sprintf(dst, "\n????: %02x", xxx);
  if (logData)
  {
    dst += sprintf(dst, "\nnumRecs=%d, nextRec=%d, RSSI=%d, sA=%d, sB=%d, R=%d", logData->numRecs, logData->nextRec, WiFi.RSSI(), SerA_cnt, SerB_cnt, Parameter.resetCount);
  }
}

// processMsg() wertet das Trimatik-Datenpaket 'src' aus
// ggfs. wird ein Eintrag im Ringpuffer und erstellt
// Info-Text in 'dst' mit dem Zustand wird aufgebaut (für WebSocket)
void processMsg(char *dst, unsigned char *src, int len)
{
  int i, start = 0;
  dst += sprintf(dst, "%ds: ", millis() / 1000);

  while (start < len)
  {
    if (src[start] != 0) break;
    start++;
  }

  for (i = start; i < len; i++)
  {
    dst += sprintf(dst, "%02x ", src[i]);
  }

  // now ist der Zeitpunkt des Paketempfangs
  // wenn Ist.ts einen Wert enthält, kann ein dt berechnet werden; ansonsten wird der Wert eh ignoriert
  // wenn Ist.ts==0, dann ist es das allererste Datenpaket und es wird kein Log-Item erstellt

  i = start;
  while (i < len)
  {
    if (src[i] == 0x0f) // 3 Byte
    {
      switch (src[i + 1])
      {
        case 0x45:  // Schaltuhr (alle 1,7s)
          if (Ist.Programm != src[i + 2])
          {
            newdata = true;
            Ist.Programm = src[i + 2];
            if (Ist.ts) logValue(&src[i]);
          }
          break;
        case 0x7f:  // (alle 1,7s)
          if (Ist.Tag != src[i + 2])
          {
            newdata = true;
            Ist.Tag = src[i + 2];
            if (Ist.ts) logValue(&src[i]);
          }
          break;
        default:  // keine Aufzeichnung, nur aktueller Wert für Status
          if (xxx != src[i + 1])
          {
            newdata = true;
            xxx = src[i + 1];
          }
          break;
      }
      i += 3;
    }
    else if (src[i] == 0x1f) // 3 Byte
    {
      switch (src[i + 1])
      {
        case 0x25:  // Ausgänge (alle 11,9s)
          if (Ist.Relais != src[i + 2])
          {
            newdata = true;
            Ist.Relais = src[i + 2];
            if (Ist.ts) logValue(&src[i]);
          }
          break;
        case 0x26:  // (alle 11,9s) - keine Aufzeichnung, nur aktueller Wert für Status
          if (Ist.x26 != src[i + 2])
          {
            newdata = true;
            Ist.x26 = src[i + 2];
#ifdef LOGALL
            if (Ist.ts) logValue(&src[i]);
#else
            if (logData) logData->Start.x26 = src[i + 2];
#endif
          }
          break;
        case 0x2F:  // Status (alle 1,7s)
          src[i + 2] += Parameter.resetCount;
          if (!logRealTS) src[i + 2] |= 0x80;
          if (Ist.Status != src[i + 2])
          {
            newdata = true;
            Ist.Status = src[i + 2];
            if (Ist.ts) logValue(&src[i]);
          }
          break;
        case 0x31:  // Aussentemperatur (alle 11,9s)
          if (Ist.Aussen != src[i + 2])
          {
            newdata = true;
            Ist.Aussen = src[i + 2];
            if (Ist.ts) logValue(&src[i]);
          }
          break;
        case 0x32:  // Vorlauftemperatur (alle 11,9s) - keine Aufzeichnung, nur aktueller Wert für Status
          if (Ist.Vorlauf != src[i + 2])
          {
            newdata = true;
            Ist.Vorlauf = src[i + 2];
#ifdef LOGALL
            if (Ist.ts) logValue(&src[i]);
#else
            if (logData) logData->Start.Vorlauf = src[i + 2];
#endif
          }
          break;
        case 0x33:  // Kesseltemperatur (alle 11,9s) - keine Aufzeichnung, nur aktueller Wert für Status
          if (Ist.Kessel2 != src[i + 2])
          {
            //newdata = true;
            Ist.Kessel2 = src[i + 2];
#ifdef LOGALL
            if (Ist.ts) logValue(&src[i]);
#else
            if (logData) logData->Start.Kessel2 = src[i + 2];
#endif
          }
          break;
        case 0x34:  // (alle 11,9s) - keine Aufzeichnung, nur aktueller Wert für Status
          if (Ist.x34 != src[i + 2])
          {
            newdata = true;
            Ist.x34 = src[i + 2];
#ifdef LOGALL
            if (Ist.ts) logValue(&src[i]);
#else
            if (logData) logData->Start.x34 = src[i + 2];
#endif
          }
          break;
        case 0x35:  // Warmwasserspeicher (alle 11,9s)
          if (Ist.Wasser != src[i + 2])
          {
            newdata = true;
            Ist.Wasser = src[i + 2];
            if (Ist.ts) logValue(&src[i]);
          }
          break;
        case 0x3A:  // Kesseltemperatur (alle 1,7s)
          if (Ist.Kessel != src[i + 2])
          {
            newdata = true;
            Ist.Kessel = src[i + 2];
            if (Ist.ts) logValue(&src[i]);
          }
          break;
        default: // keine Aufzeichnung, nur aktueller Wert für Status
          if ((unsigned char)xxx != src[i + 1])
          {
            newdata = true;
            xxx = src[i + 1];
          }
          break;
      }
      i += 3;
    }
    else if (src[i] == 0x3f) // 3 Byte
    {
      switch (src[i + 1])
      {
        case 0x2F:  // Status (alle 1,7s)
          if (Ist.Status != src[i + 2])
          {
            newdata = true;
            Ist.Status = src[i + 2];
            if (Ist.ts) logValue(&src[i]);
          }
          break;
        default: // keine Aufzeichnung, nur aktueller Wert für Status
          if ((unsigned char)xxx != src[i + 1])
          {
            newdata = true;
            xxx = src[i + 1];
          }
          break;
      }
      i += 3;
    }
    else i++;
  }
  // das Datenpaket wurde in die Ist-Werte und ggfs. die diffs geschrieben
  if (logData)
  {
    portENTER_CRITICAL(&timeMux);
    // wenn es nicht schon vorher in logValue() übernommen wurde, so wird jetzt Ist.ts aktualisiert
    if (!Ist.ts) Ist.ts = now;
    // wenn es das allererste Datenpaket ist, wird der Ist-Wert zum Start-Wert.
    if (logData->Start.ts == 0)memcpy(&logData->Start, &Ist, sizeof(Ist));
    portEXIT_CRITICAL(&timeMux);
  }
  status2str(dst);
}

// Debugging-Funktionen
// dumpMem() macht von der addr einen Dump von 8 Langworten
char *dumpMem(char *ptr, unsigned long int addr)  // 12+8*9+3=87  "0x12345678: 12345678...
{
  unsigned long *p = (unsigned long *)addr;
  unsigned short i;
  ptr += sprintf(ptr, "0x%08x: ", p);
  for (i = 0; i < 8; i++)
    ptr += sprintf(ptr, " %08x", *p++);
  *ptr++ = '\r';
  *ptr++ = '\n';
  *ptr = '\0';
  return ptr;
}

void printMemInfo()
{
  char txt[187], *t = txt;
  // GasCounterChgISR() liegt im IRAM im SRAM0-Block 0x40080000..0x4009FFFF
  // im ESP32 ist der Inhalt in umgekehrter Adressreihenfolge im Bereich 0x
  //t = dumpMem(t, 0x3FFE0000ul);

#define MEMTYPE MALLOC_CAP_INTERNAL
  Serial.printf("maxFree(" xstr(MEMTYPE) ")=%d\n", heap_caps_get_largest_free_block(MEMTYPE));
  heap_caps_print_heap_info(MEMTYPE);

#undef MEMTYPE
#define MEMTYPE MALLOC_CAP_EXEC
  Serial.printf("maxFree(" xstr(MEMTYPE) ")=%d\n", heap_caps_get_largest_free_block(MEMTYPE));
  heap_caps_print_heap_info(MEMTYPE);

#undef MEMTYPE
#define MEMTYPE MALLOC_CAP_32BIT
  Serial.printf("maxFree(" xstr(MEMTYPE) ")=%d\n", heap_caps_get_largest_free_block(MEMTYPE));
  heap_caps_print_heap_info(MEMTYPE);

#undef MEMTYPE
#define MEMTYPE MALLOC_CAP_8BIT
  Serial.printf("maxFree(" xstr(MEMTYPE) ")=%d\n", heap_caps_get_largest_free_block(MEMTYPE));
  heap_caps_print_heap_info(MEMTYPE);

#undef MEMTYPE
#define MEMTYPE MALLOC_CAP_DMA
  Serial.printf("maxFree(" xstr(MEMTYPE) ")=%d\n", heap_caps_get_largest_free_block(MEMTYPE));
  heap_caps_print_heap_info(MEMTYPE);

#undef MEMTYPE
#define MEMTYPE MALLOC_CAP_IRAM_8BIT
  Serial.printf("maxFree(" xstr(MEMTYPE) ")=%d\n", heap_caps_get_largest_free_block(MEMTYPE));
  heap_caps_print_heap_info(MEMTYPE);

#undef MEMTYPE
#define MEMTYPE MALLOC_CAP_RETENTION
  Serial.printf("maxFree(" xstr(MEMTYPE) ")=%d\n", heap_caps_get_largest_free_block(MEMTYPE));
  heap_caps_print_heap_info(MEMTYPE);

#undef MEMTYPE
#define MEMTYPE MALLOC_CAP_RTCRAM
  Serial.printf("maxFree(" xstr(MEMTYPE) ")=%d\n", heap_caps_get_largest_free_block(MEMTYPE));
  heap_caps_print_heap_info(MEMTYPE);

  /*char txt[16*87], *t = txt;
    t = dumpMem(t, 0x3FFE0000ul);
    t = dumpMem(t, 0x3FFE8000ul);
    t = dumpMem(t, 0x3FFF0000ul);
    t = dumpMem(t, 0x3FFF8000ul);
    t = dumpMem(t, 0x400A0000ul);
    t = dumpMem(t, 0x400A8000ul);
    t = dumpMem(t, 0x400B0000ul);
    t = dumpMem(t, 0x400B8000ul);
    t = dumpMem(t, 0x400C0000ul-64);
    t = dumpMem(t, 0x400B8000ul-64);
    t = dumpMem(t, 0x400B0000ul-64);
    t = dumpMem(t, 0x400A8000ul-64);
    t = dumpMem(t, 0x40000000ul-64);
    t = dumpMem(t, 0x3FFF8000ul-64);
    t = dumpMem(t, 0x3FFF0000ul-64);
    t = dumpMem(t, 0x3FFE8000ul-64);
    Serial.print(txt); */
}

// https://github.com/espressif/esp-idf/blob/master/components/lwip/apps/sntp/sntp.c
// https://github.com/espressif/esp-lwip/blob/b15cd2de75d408f9f813367571143b9bcff20738/src/apps/sntp/sntp.c
void printSNTPservers()
{
  Serial.println("NTP-Servers:");
  for (unsigned char idx = 0; idx < SNTP_MAX_SERVERS; idx++)
  {
    const char *sname = sntp_getservername(idx);  // sntp_servers[idx].name;
    const ip_addr_t *sip = sntp_getserver(idx);
    if (sip->type == IPADDR_TYPE_V4)
      Serial.printf("NTP[%d]: %d.%d.%d.%d\n", idx, ip4_addr1_val(sip->u_addr.ip4), ip4_addr2_val(sip->u_addr.ip4), ip4_addr3_val(sip->u_addr.ip4), ip4_addr4_val(sip->u_addr.ip4));
    else if (sip->type == IPADDR_TYPE_V6)
      Serial.printf("NTP[%d]: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n", idx, IP6_ADDR_BLOCK1(&(sip->u_addr.ip6)), IP6_ADDR_BLOCK2(&(sip->u_addr.ip6)), IP6_ADDR_BLOCK3(&(sip->u_addr.ip6)), IP6_ADDR_BLOCK4(&(sip->u_addr.ip6)),
                    IP6_ADDR_BLOCK5(&(sip->u_addr.ip6)), IP6_ADDR_BLOCK6(&(sip->u_addr.ip6)), IP6_ADDR_BLOCK7(&(sip->u_addr.ip6)), IP6_ADDR_BLOCK8(&(sip->u_addr.ip6)));
  }
}

/****************************** Callbacks Server Pages ******************************/
// Serve: Hauptseite "/"
void serveIndexPage(AsyncWebServerRequest *request) // beim ESP kein Unterschied: indexPageHTML liegt im PROGMEM, deshalb _P()
{
  DBUG(Serial.println("serveIndexPage()");)
  //AsyncWebServerResponse *response = request->beginResponse(200, "text/html", indexPageHTML);
  //response->addHeader("Server","ESP Async Web Server");
  //request->send(response);
  request->send_P(200, "text/html; charset=utf-8", indexPageHTML);
  DBUG(Serial.println("serve Done");)
}

// Serve: Beispiel "/msglist.txt"
void serveMsgList(AsyncWebServerRequest *request)
{
  DBUG(Serial.println("serveMsgList()");)
  //AsyncWebServerResponse *response = request->beginChunkedResponse("plain/text", handleMsgList);
  //response->addHeader("Access-Control-Allow-Origin", "*");
  //request->send(response);
  ////request->sendChunked("plain/text", handleMsgList);
  char txt[16];
  itoa(Ist.Gas, txt, 16);
  request->send(200, "text/plain; charset=windows-1252", txt);
  DBUG(Serial.println("serve Done");)
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Funktionen zum Download (CSV, JSON, binär)
// serveMesswerteCSV() liefert blockweise CSV-Daten, wobei der Dezimalpunkt/komma entsprechend der Accept-Language verwendet wird: Parameter "decimalComma".
// über den QueryString "?since=12345678" können alle Einträge die neuer als der Zeitpunkt 12345678 sind angefragt werden.
// die CSV-Daten werden in mehreren Blöcken erzeugt, wobei der aktuelle Zustand in den folgenden Variablen zwischengespeichert wird

// handleMW2CSVquick liefert CSV-Blöcke als Callback in beginChunkedResponse()
// die Quick-Version nutzt ein kompaktes Datenformat, kann aber nicht im Index rückwärts gehen
size_t handleMW2CSVquick(uint8_t *buffer, size_t maxLen, size_t index, MW2CSV_ctx *ctx)
{
  DBUG(Serial.printf("mw2csvQ(maxLen=%d, index=%d) ", maxLen, index);)

  if (logData == 0)
  {
    DBUG(Serial.println("logData=0 : => 0");)
    return 0;
  }
  if (logData->numRecs == 0)
  {
    DBUG(Serial.println("numRecs=0 : => 0");)
    return 0;
  }

  DBUG(Serial.printf("mIdx=%d, mToGo=%d : ", ctx->log.mIdx, ctx->log.mToGo);)

  char *ptr = (char *)buffer;

  // beim Start (index==0) wird die Überschrift ausgegeben
  if (index == 0)
  {
    #if defined(LOGALL)
      #define HEADER_CSV_DE "Sekunden;Gas;Aussen;Kessel;Wasser;Relais;Programm;x26;Status;Vorlauf;x34;Tag"
      #define HEADER_CSV_EN "Sekunden,Gas,Aussen,Kessel,Wasser,Relais,Programm,x26,Status,Vorlauf,x34,Tag"
    #else
      #define HEADER_CSV_DE "Sekunden;Gas;Aussen;Kessel;Wasser;Relais;Programm;;Status;;;Tag"
      #define HEADER_CSV_EN "Sekunden,Gas,Aussen,Kessel,Wasser,Relais,Programm,,Status,,,Tag"
    #endif
    #define HEADER_CSV_LEN (sizeof(HEADER_CSV_DE)-1)
    if (maxLen <= HEADER_CSV_LEN)
    {
      DBUG(Serial.println("maxLen<header => 0");)
      return 0;
    }
    if (ctx->decimalComma)
      strncpy(ptr, HEADER_CSV_DE, HEADER_CSV_LEN);
    else
      strncpy(ptr, HEADER_CSV_EN, HEADER_CSV_LEN);
    ptr += HEADER_CSV_LEN;
    maxLen -= HEADER_CSV_LEN;

    ctx->partIdx = 0;
    ctx->partLen = 0;
  } // end if (index == 0)
  
  // Methode:
  // Eintritt: in 'Werte' stehen die nächsten zu übertragenden Daten. mToGo gibt an wieviele Zeilen noch folgen einschl. Werte; sie kann 0 sein, trotzdem muss der Rest im Zeilenpuffer verarbeitet werden.
  // Wenn der Zeilenpuffer leer ist und mToGo>0 dann aus 'Werte' füllen und 'Werte' aktualisieren. Wenn mToGo==0 dann ferig.
  // Zeilenpuffer ganz oder teilweise übertragen. Wenn damit der 'buffer' gefüllt ist -> raus.
  do
  {
    if (ctx->partIdx == ctx->partLen) // wenn der Zeilenpuffer leer ist, dann aus 'Werte' füllen
    {
      if(ctx->log.mToGo == 0) break;     // wenn keine Werte mehr zu senden sind, dann Ende.
      ctx->partIdx = 0;
      // in part muss Platz sein
      // für 76 Bytes sizeof("\r\n1234567890;12345.67;+12.34;+12.3;+12.3;0x12;0x12;0x12;0x12;0x12;0x12;0x12")
      // für 65 Bytes sizeof("\r\n1234567890;12345.67;+12.34;+12.3;+12.3;0x12;0x12;;0x12;;;0x12")
      #if defined(LOGALL)
        ctx->partLen = sprintf(ctx->part, "\r\n%ld,%.2f,%.2f,%.1f,%.1f,0x%02hhx,0x%02hhx", ctx->log.Werte.ts, ctx->log.Werte.Gas / 100.0f, ((signed char)ctx->log.Werte.Aussen) / 4.0f, ctx->log.Werte.Kessel / 2.0f, ctx->log.Werte.Wasser / 2.0f, ctx->log.Werte.Relais, ctx->log.Werte.Programm);
        ctx->partLen += sprintf(ctx->part + ctx->partLen, ",0x%02hhx,0x%02hhx,0x%02hhx,0x%02hhx,0x%02hhx", ctx->log.Werte.x26, ctx->log.Werte.Status, ctx->log.Werte.Vorlauf, ctx->log.Werte.x34, ctx->log.Werte.Tag);
      #else
        ctx->partLen = sprintf(ctx->part, "\r\n%ld,%.2f,%.2f,%.1f,%.1f,0x%02hhx,0x%02hhx", ctx->log.Werte.ts, ctx->log.Werte.Gas / 100.0f, ((signed char)ctx->log.Werte.Aussen) / 4.0f, ctx->log.Werte.Kessel / 2.0f, ctx->log.Werte.Wasser / 2.0f, ctx->log.Werte.Relais, ctx->log.Werte.Programm);
        ctx->partLen += sprintf(ctx->part + ctx->partLen, ",,0x%02hhx,,,0x%02hhx", ctx->log.Werte.Status, ctx->log.Werte.Tag);
      #endif
      if (ctx->decimalComma)
      {
        char *c = ctx->part;
        do
        {
          if (*c == '.') *c = ',';
          else if (*c == ',') *c = ';';
          c++;
        } while (*c);
      }
      // die 'Werte' sind in der Zeile 'part', nächste 'Werte' aus Aufzeichnung ermitteln
      do
      {
        ctx->log.mToGo--;
        // wenn keine weiteren Daten im Puffer sind, dann hier beenden. Nur im 'part' sind noch Daten vorhanden.
        if (ctx->log.mToGo == 0) break;
  
        // ansonsten nächsten Record laden
        applyLogItem(&ctx->log.Werte, ctx->log.mIdx);
  
        ctx->log.mIdx++;
        if (ctx->log.mIdx == LOGMAX) ctx->log.mIdx = 0;
      } while (logData->msg[ctx->log.mIdx].dt == 0);  // wenn der nächste Record.dt==0 ist, gehört er noch dazu (wenn mToGo es erlaubt)
    }
    // Werte enthält nun den nächsten Datensatz; oder mToGo ist 0 und nur der Zeilenpuffer muss ausgegeben werden

    // im Zeilenpuffer 'part' sind Daten vorhanden
    int restLen = ctx->partLen - ctx->partIdx;
    if (maxLen <= restLen) // wenn nur die Zeile oder nur ein Teil davon in den 'buffer' passt (also kein Zeichen einer weiteren Zeile)
    {
      strncpy(ptr, ctx->part + ctx->partIdx, maxLen);
      ptr += maxLen;
      ctx->partIdx += maxLen;
      //DBUG(Serial.printf("Teil[%d] -> maxLen[0]\n", maxLen);)
      maxLen = 0;
      // hier ist der 'buffer' immer komplett gefüllt worden
    }
    else  // wenn mehr als der ganze Rest in den 'buffer' passt
    {
      strncpy(ptr, ctx->part + ctx->partIdx, restLen);
      ptr += restLen;
      ctx->partIdx = ctx->partLen;
      maxLen -= restLen;
      //DBUG(Serial.printf("Rest[%d] -> maxLen[%d] ", restLen, maxLen);)
    }
  } while(maxLen);

  DBUG(Serial.printf("=> %d\n", ptr - (char *)buffer);)
  return ptr - (char *)buffer;
}

// handleMW2JSONquick liefert JSON-Blöcke als Callback in beginChunkedResponse()
// die Quick-Version nutzt ein kompaktes Datenformat, kann aber nicht im Index rückwärts gehen
size_t handleMW2JSONquick(uint8_t *buffer, size_t maxLen, size_t index, MW2JSON_ctx *ctx)
{
  DBUG(Serial.printf("mw2jsonQ(maxLen=%d, index=%d) ", maxLen, index);)

  if (logData == 0)
  {
    DBUG(Serial.println("logData=0 : => 0");)
    return 0;
  }
  if (logData->numRecs == 0)
  {
    DBUG(Serial.println("numRecs=0 : => 0");)
    return 0;
  }

  DBUG(Serial.printf("mIdx=%d, mToGo=%d, logData->numRecs=%d : ", ctx->log.mIdx, ctx->log.mToGo, logData->numRecs);)

  char *ptr = (char *)buffer;

  // es werden der Start-Wert und die Diff-Werte ausgegeben
  if (index == 0)
  {
    if (maxLen <= 2)
    {
      DBUG(Serial.println("maxLen<header => 0");)
      return 0;  // es muss Platz für mind. "[]" sein.
    }
    *ptr++ = '[';
    maxLen --;
    ctx->partIdx = 0;
    ctx->partLen = 0;
  }

  do
  {
    if (ctx->partIdx == ctx->partLen) // wenn der Zeilenpuffer leer ist, dann aus 'Werte' füllen
    {
      if(ctx->log.mToGo == 0) break;     // wenn keine Werte mehr zu senden sind, dann Ende.
      ctx->partIdx = 0;
      // https://en.cppreference.com/w/c/io/fprintf
      // part braucht Platz für
      // für 70 Bytes sizeof("[1234567890,1234567,-123,123,123,0x12,0x12,0x12,0x12,0x12,0x12,0x12],") bzw.
      // für 58 Bytes sizeof("[1234567890,1234567,-123,123,123,0x12,0x12,,0x12,,,0x12],")
      #if defined(LOGALL)
        ctx->partLen = sprintf(ctx->part, "[%ld,%lu,%hhd,%hhu,%hhu,0x%02hhx,0x%02hhx", ctx->log.Werte.ts, ctx->log.Werte.Gas, (signed char)ctx->log.Werte.Aussen, ctx->log.Werte.Kessel, ctx->log.Werte.Wasser, ctx->log.Werte.Relais, ctx->log.Werte.Programm);
        ctx->partLen += sprintf(ctx->part + ctx->partLen, ",0x%02hhx,0x%02hhx,0x%02hhx,0x%02hhx,0x%02hhx],", ctx->log.Werte.x26, ctx->log.Werte.Status, ctx->log.Werte.Vorlauf, ctx->log.Werte.x34, ctx->log.Werte.Tag);
      #else
        ctx->partLen = sprintf(ctx->part, "[%ld,%lu,%hhd,%hhu,%hhu,0x%02hhx,0x%02hhx", ctx->log.Werte.ts, ctx->log.Werte.Gas, (signed char)ctx->log.Werte.Aussen, ctx->log.Werte.Kessel, ctx->log.Werte.Wasser, ctx->log.Werte.Relais, ctx->log.Werte.Programm);
        ctx->partLen += sprintf(ctx->part + ctx->partLen, ",,0x%02hhx,,,0x%02hhx],", ctx->log.Werte.Status, ctx->log.Werte.Tag);
      #endif
      if (ctx->log.mToGo == 1) ctx->part[ctx->partLen - 1] = ']';  // wenn es die letzte Zeile ist, dann "Klammer Zu"
      // die 'Werte' sind in der Zeile 'part', nächste 'Werte' aus Aufzeichnung ermitteln
      do
      {
        ctx->log.mToGo--;
        // wenn keine weiteren Daten im Puffer sind, dann hier beenden. Nur im 'part' sind noch Daten vorhanden.
        if (ctx->log.mToGo == 0) break;
  
        // ansonsten nächsten Record laden
        applyLogItem(&ctx->log.Werte, ctx->log.mIdx);
  
        ctx->log.mIdx++;
        if (ctx->log.mIdx == LOGMAX) ctx->log.mIdx = 0;
      } while (logData->msg[ctx->log.mIdx].dt == 0);  // wenn der nächste Record.dt==0 ist, gehört er noch dazu (wenn mToGo es erlaubt)
    }
    // Werte enthält nun den nächsten Datensatz; oder mToGo ist 0 und nur der Zeilenpuffer muss ausgegeben werden

    // im Zeilenpuffer 'part' sind Daten vorhanden
    int restLen = ctx->partLen - ctx->partIdx;
    if (maxLen <= restLen) // wenn nur die Zeile oder nur ein Teil davon in den 'buffer' passt (also kein Zeichen einer weiteren Zeile)
    {
      strncpy(ptr, ctx->part + ctx->partIdx, maxLen);
      ptr += maxLen;
      ctx->partIdx += maxLen;
      DBUG(Serial.printf("Teil[%d] -> maxLen[0]\n", maxLen);)
      maxLen = 0;
      // hier ist der 'buffer' immer komplett gefüllt worden
    }
    else  // wenn mehr als der ganze Rest in den 'buffer' passt
    {
      strncpy(ptr, ctx->part + ctx->partIdx, restLen);
      ptr += restLen;
      ctx->partIdx = ctx->partLen;
      maxLen -= restLen;
      //DBUG(Serial.printf("Rest[%d] -> maxLen[%d] ", restLen, maxLen);)
    }
  } while(maxLen);

  DBUG(Serial.printf("=> %d\n", ptr - (char *)buffer);)
  return ptr - (char *)buffer;
}

// handleMW2BINquick() liefert blockweise binäre Daten für das Diagramm des WebInterfaces als Callback in beginChunkedResponse()
// der Kontext ctx enthält den Start-Index und die Anzahl der zu übertragenden Elemente mToGo, sowie den aktuellen Werte-Zustand
size_t handleMW2BINquick(uint8_t *buffer, size_t maxLen, size_t index, logCtx *ctx)
{
  // im BIN ist der index direkt übersetzbar. Das Format der BIN-Datei ist:
  // 0: 4 Bytes Werte.ts
  // 4: 4 Bytes Werte.Gas
  // 8: 10x4 Bytes für die Startwerte von Werte.Aussen, Werte.Kessel, Werte.Wasser, Werte.Relais, Werte.Programm, Werte.x26, Werte.Status, Werte.Vorlauf, Werte.x34, Werte.Tag
  // oder
  // 8: 7x4 Bytes für die Startwerte von Werte.Aussen, Werte.Kessel, Werte.Wasser, Werte.Relais, Werte.Programm, Werte.Status, Werte.Tag
  // 48 bzw. 36: numRecs*4 Bytes Records

  DBUG(Serial.printf("mw2binQ(maxLen=%d, index=%d) ", maxLen, index);)

  if (logData == 0)
  {
    DBUG(Serial.println("logData=0 : => 0");)
    return 0;
  }
  if (logData->numRecs == 0)
  {
    DBUG(Serial.println("numRecs=0 : => 0");)
    return 0;
  }

  trimatikRecord *rec = (trimatikRecord *)buffer;

  // wenn Start der Ausgabe, dann den Index auf ersten Wert setzen
  if (index == 0)
  {
#if defined(LOGALL)
    if (maxLen < 48) // Start-Datensatz aus ts, Gas und 10 Elementen
    {
      DBUG(Serial.println("maxLen<48 : => 0");)
      return 0;
    }
#else
    if (maxLen < 36) // Start-Datensatz aus ts, Gas und 7 Elementen
    {
      DBUG(Serial.println("maxLen<36 : => 0");)
      return 0;
    }
#endif

    // Header schreiben
    *(time_t *)buffer = ctx->Werte.ts;
    *(unsigned long int *)(buffer + sizeof(time_t)) = ctx->Werte.Gas;
    rec = (trimatikRecord *)(buffer + sizeof(time_t) + sizeof(unsigned long int));
    // 8: 10x4 Bytes für die Startwerte von Werte.Aussen, Werte.Kessel, Werte.Wasser, Werte.Relais, Werte.Programm, Werte.x26, Werte.Status, Werte.Vorlauf, Werte.x34, Werte.Tag
    // 8: 7x4 Bytes für die Startwerte von Werte.Aussen, Werte.Kessel, Werte.Wasser, Werte.Relais, Werte.Programm, Werte.Status, Werte.Tag
    rec->dt = 0;
    rec->id = 0x31;
    rec->value = ctx->Werte.Aussen;
    rec++;
    rec->dt = 0;
    rec->id = 0x3a;
    rec->value = ctx->Werte.Kessel;
    rec++;
    rec->dt = 0;
    rec->id = 0x35;
    rec->value = ctx->Werte.Wasser;
    rec++;
    rec->dt = 0;
    rec->id = 0x25;
    rec->value = ctx->Werte.Relais;
    rec++;
    rec->dt = 0;
    rec->id = 0x45;
    rec->value = ctx->Werte.Programm;
    rec++;
#if defined(LOGALL)
    rec->dt = 0;
    rec->id = 0x26;
    rec->value = ctx->Werte.x26;
    rec++;
#endif
    rec->dt = 0;
    rec->id = 0x2f;
    rec->value = ctx->Werte.Status;
    rec++;
#if defined(LOGALL)
    rec->dt = 0;
    rec->id = 0x32;
    rec->value = ctx->Werte.Vorlauf;
    rec++;
    rec->dt = 0;
    rec->id = 0x34;
    rec->value = ctx->Werte.x34;
    rec++;
#endif
    rec->dt = 0;
    rec->id = 0x7f;
    rec->value = ctx->Werte.Tag;
    rec++;

    maxLen -= ((uint8_t *)rec - buffer);
    if(ctx->mToGo) ctx->mToGo--;  // den Startwert von der Anzahl abziehen
  }

  DBUG(Serial.printf("mIdx=%d, mToGo=%d, logData->numRecs=%d : ", ctx->mIdx, ctx->mToGo, logData->numRecs);)

  // https://en.cppreference.com/w/c/io/fprintf
  while (ctx->mToGo && maxLen >= sizeof(trimatikRecord)) // solange Records vorhanden sind und Platz im Puffer ist, wird der Datensatz aus 'Werte' gesendet
  {
    *rec = logData->msg[ctx->mIdx];
    rec++;
    ctx->mIdx++;
    if (ctx->mIdx == LOGMAX) ctx->mIdx = 0;
    maxLen -= sizeof(trimatikRecord);
    ctx->mToGo--;
  } // end while(ctx->mToGo && maxLen>4)

  DBUG(Serial.printf("=> %d\n", (uint8_t *)rec - buffer);)
  return (uint8_t *)rec - buffer;
}

// erzeugt eine Liste der WiFis-Netze
// beim ersten Aufruf wird der Scan angestoßen und "scanning..." zurückgegeben.
// beim nächsten Aufruf wird das Scan-Ergebnis geliefert
// die Quick-Version nutzt ein kompaktes Datenformat, kann aber nicht im Index rückwärts gehen
size_t handleWiFiScan(uint8_t *buffer, size_t maxLen, size_t index, MW2JSON_ctx *ctx)
{
  char *ptr = (char *)buffer;

  DBUG(Serial.printf("WiFiScanQ(maxLen=%d, index=%d) ", maxLen, index);)

  // wenn Start der Ausgabe, dann Suche Ausführen
  // int16_t scanComplete() hat diese Rückgabewerte:
  // 0 oder positiv: Anzahl Netzwerke
  // WIFI_SCAN_RUNNING -1
  // WIFI_SCAN_FAILED -2
  int16_t scan = WiFi.scanComplete();
  if (scan == WIFI_SCAN_RUNNING)  // Scanvorgang läuft, nur bei erstem Ausruf etwas zurückgeben
  {
    if (index == 0)
    {
      #define WIFISCAN_TEXT_SCANNING "scanning..."
      strncpy(ptr, WIFISCAN_TEXT_SCANNING, sizeof(WIFISCAN_TEXT_SCANNING) - 1);
      ptr += (sizeof(WIFISCAN_TEXT_SCANNING) - 1);
      DBUG(Serial.print(" - scanning -");)
    }
  }
  else if (scan == WIFI_SCAN_FAILED) // kein Ergebnis: Scan (neu) ausführen
  {
    if (index == 0)
    {
      DBUG(Serial.print(" - start scan ");)
      int16_t start = WiFi.scanNetworks(true);  // async scan
      if (start == WIFI_SCAN_RUNNING)
      {
        #define WIFISCAN_TEXT_STARTSCAN "Scan started."
        strncpy(ptr, WIFISCAN_TEXT_STARTSCAN, sizeof(WIFISCAN_TEXT_STARTSCAN) - 1);
        ptr += (sizeof(WIFISCAN_TEXT_STARTSCAN) - 1);
        DBUG(Serial.print("ok ");)
      }
      else
      {
        #define WIFISCAN_TEXT_SCANFAILED "Scan failed."
        strncpy(ptr, WIFISCAN_TEXT_SCANFAILED, sizeof(WIFISCAN_TEXT_SCANFAILED) - 1);
        ptr += (sizeof(WIFISCAN_TEXT_SCANFAILED) - 1);
        DBUG(Serial.print("failed ");)
      }
    }
  }
  else      // es wurden 0 oder mehr Netze gefunden
  {
    if (index == 0)
    {
      ctx->log.mToGo = scan;
      ctx->log.mIdx = 0;
      #define WIFISCAN_TEXT_HEADER "Nr | SSID                             | RSSI | CH | Encryption\n"
      strncpy(ctx->part, WIFISCAN_TEXT_HEADER, sizeof(WIFISCAN_TEXT_HEADER) - 1);
      ctx->partIdx = 0;
      ctx->partLen = sizeof(WIFISCAN_TEXT_HEADER) - 1;
    }

    DBUG(Serial.printf("mIdx=%d, mToGo=%d : ", ctx->log.mIdx, ctx->log.mToGo);)

    do
    {
      int restLen = ctx->partLen - ctx->partIdx;
      if (restLen > 0)
      {
        if (restLen <= maxLen) // wenn der ganze Rest in den Puffer passt
        {
          strncpy(ptr, ctx->part + ctx->partIdx, restLen);
          ptr += restLen;
          ctx->partIdx = ctx->partLen;
          maxLen -= restLen;
          DBUG(Serial.printf("[%d] ", restLen);)
        }
        else // wenn nur ein Teilstring (ohne Null) in den Puffer passt
        {
          strncpy(ptr, ctx->part + ctx->partIdx, maxLen);
          ptr += maxLen;
          ctx->partIdx += maxLen;
          DBUG(Serial.printf("[%d] => %d\n", maxLen, ptr - (char *)buffer);)
          maxLen = 0;
          break;
        }
      }

      if (ctx->log.mToGo == 0)
      {
        // das war alles
        WiFi.scanDelete();
        break;
      }
      //                      "12 | 12345678901234567890123456789012 | +123 | 12 | WPA2_ENTERPRISE\n\0"
      ctx->partLen = sprintf(ctx->part, "%2d | %-32.32s | %4d | %2d | %s\n", ctx->log.mIdx + 1, WiFi.SSID(ctx->log.mIdx).c_str(), WiFi.RSSI(ctx->log.mIdx), WiFi.channel(ctx->log.mIdx), translateEncryptionType(WiFi.encryptionType(ctx->log.mIdx)));
      ctx->partIdx = 0;
      ctx->log.mToGo--;
      ctx->log.mIdx++;
    } while(maxLen > 0); // solange Platz im Puffer ist
  }

  DBUG(Serial.printf("=> %d\n", ptr - (char *)buffer);)
  return ptr - (char *)buffer;
}

// initCtx() bereitet einen Kontext für die Request-Callbacks vor:
// setzt den Startwert von mIdx (Element-Index) und mToGo (Anzahl Elemente) für die Elemente nach sinceTS
// und füllt 'Werte' mit dem Startzustand
void initCtx(logCtx *ctx, time_t sinceTS)
{
  ctx->mToGo = 0;
  if(logData == 0) return;
  // Startindex bzw. Korrektur des Messwertindexes im Ringpuffer
  if(logData->numRecs == LOGMAX)
  {
    ctx->mIdx = logData->nextRec + 1; // der nextRec ist mit dem Magic belegt, also überspringen
    if(ctx->mIdx == LOGMAX) ctx->mIdx = 0;
    ctx->mToGo = LOGMAX - 1;
  }
  else
  {
    // Startindex
    ctx->mIdx = 0;
    // Anzahl der vorhandenen Werte im Ringpuffer
    ctx->mToGo = logData->numRecs;
  }
  ctx->Werte = logData->Start;  // Start-Zustand kopieren; wird beim Durchlaufen des Puffers aktualisiert
  while (ctx->mToGo && ctx->Werte.ts <= sinceTS)
  {
    do
    {
      // nächsten Record laden
      applyLogItem(&ctx->Werte, ctx->mIdx);
      ctx->mIdx++;
      if (ctx->mIdx == LOGMAX) ctx->mIdx = 0;
      ctx->mToGo--;

      // wenn keine weiteren Daten im Puffer sind, dann hier beenden.
      if (ctx->mToGo == 0)
      {
        if (ctx->Werte.ts <= sinceTS)
        {
          DBUG(Serial.println("skipping all");)
          return;
        }
        break;
      }
    } while (logData->msg[ctx->mIdx].dt == 0);  // wenn der nächste Record.dt==0 ist, gehört er noch dazu (wenn mToGo es erlaubt)
  }
  // Anzahl der auszugebenden Messwerte einschließlich Start-Set
  ctx->mToGo++;
}

// Serve: "messwerte.json"
void serveMesswerteJSON(AsyncWebServerRequest *request)
{
  DBUG(Serial.println("serveMesswerteJSON()");)
  // JSON soll UTF-8 sein
  time_t sinceTS = 0;
  AsyncWebParameter* p = request->getParam("since");
  if (p != NULL)
  {
    sinceTS = atoi(p->value().c_str());
  }
  MW2JSON_ctx ctx;
  initCtx(&ctx.log, sinceTS);
  ctx.partIdx = 0;
  ctx.partLen = 0;
  
  AsyncWebServerResponse *response = request->beginChunkedResponse("application/json",
    [ctx](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t
    {
      //DBUG(Serial.printf("lambda - &ctx=0x%08x\n",&ctx);)
      return handleMW2JSONquick(buffer, maxLen, index, &ctx);
    });
  response->addHeader("Access-Control-Allow-Origin", "*");
  if (logData)
  {
    if (logData->numRecs != 0)
    {
      struct tm timeinfo;
      localtime_r(&Ist.ts, &timeinfo);
      char result[30];
      strftime (result, 30, "%d %b %Y %H:%M:%S %z", &timeinfo); // "13 Jun 2023 08:39:31 +0200"
      DBUG(Serial.println(result);)
      response->addHeader("Last-Modified", result);
    }
  }
  request->send(response);
  //request->sendChunked("application/json", handlePM2JSON);
  DBUG(Serial.println("serve Done");)
}
// Serve: "messwerte.csv"
void serveMesswerteCSV(AsyncWebServerRequest *request)
{
  DBUG(Serial.println("serveMesswerteCSV()");)
  //List all collected headers
  //for(int i=0, headers = request->headers(); i<headers; i++)
  //{
  //  AsyncWebHeader* h = request->getHeader(i);
  //  Serial.printf("HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
  //}
  /*struct tm modTime;
    if(AsyncWebHeader* h = request->getHeader("If-Modified-Since"))
    {
    // int   getdate_r (h->value().c_str(), &modTime);
    strptime(h->value().c_str(), "%a, %d %b %Y %H:%M:%S %Z", &modTime);
    }*/
  time_t sinceTS = 0;
  AsyncWebParameter* p = request->getParam("since");
  if (p != NULL)
  {
    sinceTS = atoi(p->value().c_str());
  }
  MW2CSV_ctx ctx;
  initCtx(&ctx.log, sinceTS);
  ctx.decimalComma = false;
  if (AsyncWebHeader* h = request->getHeader("Accept-Language"))
  {
    const char *lang = h->value().c_str();
    if (lang[0] == 'd' && lang[1] == 'e') ctx.decimalComma = true;
  }
  ctx.partIdx = 0;
  ctx.partLen = 0;
  
  AsyncWebServerResponse *response = request->beginChunkedResponse("text/csv", 
    [ctx](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t
    {
      //DBUG(Serial.printf("lambda - &ctx=0x%08x\n",&ctx);)
      return handleMW2CSVquick(buffer, maxLen, index, &ctx);
    });

  response->addHeader("Access-Control-Allow-Origin", "*");
  //response->addHeader("Content-Disposition", "attachment; filename="messwerte.csv");
  response->addHeader("Content-Disposition", "inline; filename=\"messwerte.csv\"");
  if (logData)
  {
    if (logData->numRecs != 0)
    {
      struct tm timeinfo;
      // https://cplusplus.com/reference/ctime/strftime/

      // lt. moz: Last-Modified:"Thu, 08 Jun 2023 16:01:52 GMT"
      gmtime_r(&Ist.ts, &timeinfo);
      char result[32];
      strftime (result, 32, "%a, %d %b %Y %H:%M:%S %Z", &timeinfo); // "Thu, 13 Jun 2023 08:39:31 GMT"
      DBUG(Serial.println(result);)
      response->addHeader("Last-Modified", result);
    }
  }
  request->send(response);
  DBUG(Serial.println("serve Done");)
}

// Serve: "messwerte.bin"
void serveMesswerteBIN(AsyncWebServerRequest *request)
{
  DBUG(Serial.println("serveMesswerteBIN()");)
  time_t sinceTS = 0;
  AsyncWebParameter* p = request->getParam("since");
  if (p != NULL)
  {
    sinceTS = atoi(p->value().c_str());
  }
  logCtx ctx;
  initCtx(&ctx, sinceTS);
  
  AsyncWebServerResponse *response = request->beginChunkedResponse("application/octet-stream", 
    [ctx](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t
    {
      //DBUG(Serial.printf("lambda - &ctx=0x%08x\n",&ctx);)
      return handleMW2BINquick(buffer, maxLen, index, &ctx);
    });

  response->addHeader("Access-Control-Allow-Origin", "*");
  if (logData)
  {
    if (logData->numRecs != 0)
    {
      struct tm timeinfo;
      localtime_r(&Ist.ts, &timeinfo);
      char result[30];
      strftime (result, 30, "%d %b %Y %H:%M:%S %z", &timeinfo); // "13 Jun 2023 08:39:31 +0200"
      DBUG(Serial.println(result);)
      response->addHeader("Last-Modified", result);
    }
  }
  request->send(response);
  //request->sendChunked("application/octet-stream", handlePM2BIN);
  DBUG(Serial.println("serve Done");)
}
// Serve: "/wifiscan.txt"
void serveWiFiScan(AsyncWebServerRequest *request)
{
  DBUG(Serial.println("serveWiFiScan()");)

  MW2JSON_ctx ctx;
  initCtx(&ctx.log, 0);
  ctx.partIdx = 0;
  ctx.partLen = 0;
  
  AsyncWebServerResponse *response = request->beginChunkedResponse("text/plain",
    [ctx](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t
    {
      //DBUG(Serial.printf("lambda - &ctx=0x%08x\n",&ctx);)
      return handleWiFiScan(buffer, maxLen, index, &ctx);
    });
  response->addHeader("Access-Control-Allow-Origin", "*");
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char result[30];
  strftime (result, 30, "%d %b %Y %H:%M:%S %z", &timeinfo); // "13 Jun 2023 08:39:31 +0200"
  response->addHeader("Last-Modified", result);
  request->send(response);
  DBUG(Serial.println("serve Done");)
}
// Serve: "/info.txt"
void serveInfoTxt(AsyncWebServerRequest *request)
{
  DBUG(Serial.println("serveInfoText()");)
  char txt[1024];
  char *ptr = txt;

  ptr += sprintf(ptr, "getXtalFrequencyMhz() = %d MHz\n", getXtalFrequencyMhz());
  ptr += sprintf(ptr, "getCpuFrequencyMhz() = %d MHz\n", getCpuFrequencyMhz());
  ptr += sprintf(ptr, "getApbFrequency() = %d Hz = %d MHz\n", getApbFrequency(), getApbFrequency() / 1000000UL);
  ptr += sprintf(ptr, "ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());

  ptr += sprintf(ptr, "ESP.getCpuFreqMHz() = %d MHz\n", ESP.getCpuFreqMHz());

  ptr += sprintf(ptr, "ESP.getSdkVersion() = %s\n", ESP.getSdkVersion());

  ptr += sprintf(ptr, "ESP.getFlashChipSize() = %d\n", ESP.getFlashChipSize());
  ptr += sprintf(ptr, "ESP.getFlashChipSpeed() = %d\n", ESP.getFlashChipSpeed());

  ptr += sprintf(ptr, "ESP.getHeapSize() = %u\n", ESP.getHeapSize());
  ptr += sprintf(ptr, "esp_get_minimum_free_heap_size() = %u\n", esp_get_minimum_free_heap_size());
  ptr += sprintf(ptr, "heap_caps_get_free_size(8BIT INTERNAL) = %d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
  ptr += sprintf(ptr, "heap_caps_get_largest_free_block(8BIT INTERNAL) = %u\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
  //ptr += sprintf(ptr,"ESP.getFreeHeap() = %d\n",ESP.getFreeHeap());
  //ptr += sprintf(ptr,"ESP.getMaxAllocHeap() = %u\n", ESP.getMaxAllocHeap());
  ptr += sprintf(ptr, "ESP.getFreeSketchSpace() = %d\n", ESP.getFreeSketchSpace());
  ptr += sprintf(ptr, "ESP.getSketchSize() = %d\n", ESP.getSketchSize());
  uint16_t cores = ESP.getChipCores();
  ptr += sprintf(ptr, "ESP.getChipCores() = %d\n", cores);
  for (uint16_t c = 0; c < cores; c++)
  {
    RESET_REASON reason = rtc_get_reset_reason(c);
    ptr += sprintf(ptr, "Reset reason CPU%d: %s %s\n", c, reset_reason(reason), verbose_reset_reason(reason));
  }
  time_t resetTS = now - millis() / 1000;
  ptr += sprintf(ptr, "Reset# %d on %s", Parameter.resetCount, asctime (localtime (&resetTS)));
  ptr += sprintf(ptr, "MAC: %s\n", WiFi.macAddress().c_str());
  const ip_addr_t *sip = sntp_getserver(0);
  ptr += sprintf(ptr, "SNTP: ");
  if (sip->type == IPADDR_TYPE_V4)
    ptr += sprintf(ptr, "%d.%d.%d.%d", ip4_addr1_val(sip->u_addr.ip4), ip4_addr2_val(sip->u_addr.ip4), ip4_addr3_val(sip->u_addr.ip4), ip4_addr4_val(sip->u_addr.ip4));
  if (sip->type == IPADDR_TYPE_V6)
  {
    ptr += sprintf(ptr, "%04x:%04x:%04x:%04x:", IP6_ADDR_BLOCK1(&(sip->u_addr.ip6)), IP6_ADDR_BLOCK2(&(sip->u_addr.ip6)), IP6_ADDR_BLOCK3(&(sip->u_addr.ip6)), IP6_ADDR_BLOCK4(&(sip->u_addr.ip6)));
    unsigned int block;
    block = IP6_ADDR_BLOCK5(&(sip->u_addr.ip6));
    if (block != 0) ptr += sprintf(ptr, "%04x", block);
    *ptr++ = ':';
    block = IP6_ADDR_BLOCK6(&(sip->u_addr.ip6));
    if (block != 0) ptr += sprintf(ptr, "%04x", block);
    *ptr++ = ':';
    block = IP6_ADDR_BLOCK7(&(sip->u_addr.ip6));
    if (block != 0) ptr += sprintf(ptr, "%04x", block);
    *ptr++ = ':';
    block = IP6_ADDR_BLOCK8(&(sip->u_addr.ip6));
    if (block != 0) ptr += sprintf(ptr, "%04x", block);
  }
  const char *sname = sntp_getservername(0);  // sntp_servers[idx].name;
  if (sname != NULL) ptr += sprintf(ptr, " - %s", sname);
  *ptr++ = '\n';
  ptr += sprintf(ptr, "NTP sync# %d on %s", ntpSyncCount, asctime (localtime (&ntpSyncTS)));
  ptr += sprintf(ptr, "now: %s", asctime (localtime (&now)));

  ptr += sprintf(ptr, "Chip-Temperatur: %.1f""\xB0""C\n", temperatureRead());
  ptr += sprintf(ptr, "psramFound() = %u\n", psramFound());
  if (psramFound())
  {
    //ptr += sprintf(ptr,"esp_spiram_get_size() = %u\n", esp_spiram_get_size());   // bleibt hängen
    ptr += sprintf(ptr, "ESP.getPsramSize() = %u\n", ESP.getPsramSize());
    ptr += sprintf(ptr, "heap_caps_get_free_size(MALLOC_CAP_SPIRAM) = %u\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ptr += sprintf(ptr, "heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) = %u\n", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    //ptr += sprintf(ptr,"ESP.getMaxAllocPsram() = %u\n", ESP.getMaxAllocPsram());
    //ptr += sprintf(ptr,"ESP.getFreePsram() = %u\n", ESP.getFreePsram());
  }
  ptr += sprintf(ptr, "logData(%d) = 0x%08x\n", LOGMEMSIZE, logData);

  // Strings werden in UTF-8 erzeugt, d.h. '°' wird zu 0xC2B0
  // in ANSI / windows-1252 encodierung ist '°' 0xB0
  //AsyncWebServerResponse *response = request->beginResponse(200, "text/plain; charset=utf-8", txt);
  //response->addHeader("Server","ESP Async Web Server");
  //request->send(response);
  request->send(200, "text/plain; charset=windows-1252", txt);
  DBUG(Serial.println("serve Done");)
}
// Serve: "/param.json"
void serveParamJSON(AsyncWebServerRequest *request)
{ // '{"Tag":-128,"Nacht":-128,"Wasser":-128,"Neigung":-128,"Niveau":-128,"Gas":1234567}\0'
  DBUG(Serial.println("serveParamJSON()");)
  char json[96];
  sprintf(json, "{\"Tag\":%hhd,\"Nacht\":%hhd,\"Wasser\":%hhd,\"Neigung\":%hhu,\"Niveau\":%hhd,\"Gas\":%u}", Parameter.TempTag, Parameter.TempNacht, Parameter.TempWasser, Parameter.Neigung, Parameter.Niveau, Ist.Gas);
  // Strings werden in UTF-8 erzeugt, d.h. '°' wird zu 0xC2B0
  // in ANSI / windows-1252 encodierung ist '°' 0xB0
  //AsyncWebServerResponse *response = request->beginResponse(200, "text/plain; charset=utf-8", txt);
  //response->addHeader("Server","ESP Async Web Server");
  //request->send(response);
  //request->send(200, "text/plain; charset=windows-1252", json);

  AsyncWebServerResponse *response = request->beginResponse(200, "text/plain; charset=utf-8", json);
  response->addHeader("Connection", "close");
  response->addHeader("Access-Control-Allow-Origin", "*");
  request->send(response);
  DBUG(Serial.println("serve Done");)
}

/*
         ts           Gas     AT    KT   KT2  WW   Rl Pr St Ck VT 26 34
   Start 1732048755 12345.67 -12.25 12.5 12.5 12.5 ab 55 00 84 ff 13 ff
   Log   1732048755 12345.67 -12.25 12.5 12.5 12.5 ab 55 00 84 ff 13 ff
   Ist   1732048755 12345.67 -12.25 12.5 12.5 12.5 ab 55 00 84 ff 13 ff

         ts       Gas      Rl St AT WW KT Pr Ck 26 VT K2 34
   Start 673cf773 00000000 00 00 00 00 00 84 00 44 00 ff 00 00
   Log   673cf773 00000000 00 00 00 00 00 84 00 44 00 ff 00 00
   Ist   673cf773 00000000 00 00 00 00 00 84 00 44 00 ff 00 00

   Log consistent: false
*/
// Serve: "/logreport.txt"
void serveLogReport(AsyncWebServerRequest *request)
{
  DBUG(Serial.println("serveLogReport()");)
  if (logData)
  {
    char report[560];
    char *ptr = report;

    #define TEXT_LOGREPORT_VALHEADER "      ts            Gas     AT    KT  KT2   WW  Rl Pr St Ck VT 26 34\n"
    strcpy(ptr, TEXT_LOGREPORT_VALHEADER);
    ptr += sizeof(TEXT_LOGREPORT_VALHEADER) - 1;

    Werte = logData->Start;  // Start-Zustand kopieren
    strcpy(ptr, "Start ");
    ptr += 6;
    ptr += sprintf(ptr, "%10d %8.2f %6.2f %4.1f %4.1f %4.1f %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx\n", Werte.ts, Werte.Gas / 100.0f, ((signed char)Werte.Aussen) / 4.0f, Werte.Kessel / 2.0f, Werte.Kessel2 / 2.0f,
                   Werte.Wasser / 2.0f, Werte.Relais, Werte.Programm, Werte.Status, Werte.Tag, Werte.Vorlauf, Werte.x26, Werte.x34);
    //ptr += sizeof("1732048755 12345.67 -12.25 12.5 12.5 12.5 ab 55 00 84 ff 13 ff\n")-1;

    applyLog(&Werte);

    strcpy(ptr, "Log   ");
    ptr += 6;
    ptr += sprintf(ptr, "%10d %8.2f %6.2f %4.1f %4.1f %4.1f %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx\n", Werte.ts, Werte.Gas / 100.0f, ((signed char)Werte.Aussen) / 4.0f, Werte.Kessel / 2.0f, Werte.Kessel2 / 2.0f,
                   Werte.Wasser / 2.0f, Werte.Relais, Werte.Programm, Werte.Status, Werte.Tag, Werte.Vorlauf, Werte.x26, Werte.x34);
    //ptr += sizeof("1732048755 12345.67 -12.25 12.5 12.5 12.5 ab 55 00 84 ff 13 ff\n")-1;

    strcpy(ptr, "Ist   ");
    ptr += 6;
    ptr += sprintf(ptr, "%10d %8.2f %6.2f %4.1f %4.1f %4.1f %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx\n", Ist.ts, Ist.Gas / 100.0f, ((signed char)Ist.Aussen) / 4.0f, Ist.Kessel / 2.0f, Ist.Kessel2 / 2.0f,
                   Ist.Wasser / 2.0f, Ist.Relais, Ist.Programm, Ist.Status, Ist.Tag, Ist.Vorlauf, Ist.x26, Ist.x34);
    //ptr += sizeof("1732048755 12345.67 -12.25 12.5 12.5 12.5 ab 55 00 84 ff 13 ff\n")-1;

    #define TEXT_LOGREPORT_HEXHEADER "\n      ts       Gas      Rl St AT WW KT Pr Ck 26 VT K2 34\n"
    strcpy(ptr, TEXT_LOGREPORT_HEXHEADER);
    ptr += sizeof(TEXT_LOGREPORT_HEXHEADER) - 1;

    strcpy(ptr, "Start ");
    ptr += 6;
    ptr += sprintf(ptr, "%08x %08x", logData->Start.ts, logData->Start.Gas);
    unsigned char *c = (unsigned char *)(&logData->Start.Relais);
    for (int i = 0; i < 12; i++)
      ptr += sprintf(ptr, " %02hhx", *c++);
    *ptr++ = '\n';
    // ptr += sizeof("673cf773 00000000 00 00 00 00 00 84 00 44 00 ff 00 00\n")-1;

    strcpy(ptr, "Log   ");
    ptr += 6;
    ptr += sprintf(ptr, "%08x %08x", Werte.ts, Werte.Gas);
    c = (unsigned char *)(&Werte.Relais);
    for (int i = 0; i < 12; i++)
      ptr += sprintf(ptr, " %02x", *c++);
    *ptr++ = '\n';
    // ptr += sizeof("673cf773 00000000 00 00 00 00 00 84 00 44 00 ff 00 00\n")-1;

    strcpy(ptr, "Ist   ");
    ptr += 6;
    ptr += sprintf(ptr, "%08x %08x", Ist.ts, Ist.Gas);
    c = (unsigned char *)(&Ist.Relais);
    for (int i = 0; i < 12; i++)
      ptr += sprintf(ptr, " %02x", *c++);
    *ptr++ = '\n';
    // ptr += sizeof("673cf773 00000000 00 00 00 00 00 84 00 44 00 ff 00 00\n")-1;

    #define TEXT_LOGREPORT_CONSHEADER "\nLog consistent: "
    strcpy(ptr, TEXT_LOGREPORT_CONSHEADER);
    ptr += sizeof(TEXT_LOGREPORT_CONSHEADER) - 1;

    if (memcmp(&Werte, &Ist, sizeof(Werte)) == 0)
    {
      strcpy(ptr, "true");
    }
    else
    {
      strcpy(ptr, "false");
    }
    request->send(200, "text/plain; charset=windows-1252", report);
  }
  DBUG(Serial.println("serve Done");)
}

// Serve: "/logmem.txt"
void serveLogTxt(AsyncWebServerRequest *request)
{
  char txt[3 * 87], *t = txt;
  t = dumpMem(t, ((unsigned long int)logData));
  t = dumpMem(t, ((unsigned long int)logData) + 32);
  t = dumpMem(t, ((unsigned long int)logData) + 64);
  request->send(200, "text/plain; charset=windows-1252", txt);
}

// Serve: "/ram.txt"
void serveRAMTxt(AsyncWebServerRequest *request)
{
  char txt[16 * 87], *t = txt;
  t = dumpMem(t, 0x3FFE0000ul);
  t = dumpMem(t, 0x3FFE8000ul);
  t = dumpMem(t, 0x3FFF0000ul);
  t = dumpMem(t, 0x3FFF8000ul);
  t = dumpMem(t, 0x400A0000ul);
  t = dumpMem(t, 0x400A8000ul);
  t = dumpMem(t, 0x400B0000ul);
  t = dumpMem(t, 0x400B8000ul);
  t = dumpMem(t, 0x400C0000ul - 64);
  t = dumpMem(t, 0x400B8000ul - 64);
  t = dumpMem(t, 0x400B0000ul - 64);
  t = dumpMem(t, 0x400A8000ul - 64);
  t = dumpMem(t, 0x40000000ul - 64);
  t = dumpMem(t, 0x3FFF8000ul - 64);
  t = dumpMem(t, 0x3FFF0000ul - 64);
  t = dumpMem(t, 0x3FFE8000ul - 64);
  request->send(200, "text/plain; charset=windows-1252", txt);
}

// Serve: "/psram.txt"
void servePSRAMTxt(AsyncWebServerRequest *request)
{
  char txt[640];
  char *ptr = txt;

  ptr += sprintf(ptr, "ESP.getHeapSize() = %u\n", ESP.getHeapSize());
  ptr += sprintf(ptr, "ESP.getFreeHeap() = %u\n", ESP.getFreeHeap());
  ptr += sprintf(ptr, "ESP.getMaxAllocHeap() = %u\n", ESP.getMaxAllocHeap());
  ptr += sprintf(ptr, "psramFound() = %u\n", psramFound());
  if (psramFound())
  {
    ptr += sprintf(ptr, "ESP.getPsramSize() = %u\n", ESP.getPsramSize());
    ptr += sprintf(ptr, "ESP.getFreePsram() = %u\n", ESP.getFreePsram());
    ptr += sprintf(ptr, "ESP.getMaxAllocPsram() = %u\n\n", ESP.getMaxAllocPsram());
  }

#ifdef DYNAMIC_LOGMEM
  if (logData == 0)
  {
    // heap_caps_malloc_prefer( LOGMEMSIZE, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT );
    #ifdef USE_PSRAM
      if (psramFound())
      {
        logData = (trimatikLog *)ps_malloc(LOGMEMSIZE);
        ptr += sprintf(ptr, "ps_malloc(%d) = 0x%08x\n", LOGMEMSIZE, logData);
      }
    #endif
    #ifdef FORCE_DRAM
      if (logData == 0)
      {
        logData = (trimatikLog *)heap_caps_malloc(LOGMEMSIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ptr += sprintf(ptr, "heap_caps_malloc(%d) = 0x%08x\n", LOGMEMSIZE, logData);
      }
    #else
      if (logData == 0)
      {
        logData = (trimatikLog *)malloc(LOGMEMSIZE);
        ptr += sprintf(ptr, "malloc(%d) = 0x%08x\n", LOGMEMSIZE, logData);
      }
    #endif
    if (logData != 0)
    {
      ptr += sprintf(ptr, "magic_start = 0x%08x (0x%08x), numRecs = 0x%08x, nextRec = 0x%08x, msg[0].dt = 0x%08x\n", logData->magic_start, LOGMAGIC_START, logData->numRecs, logData->nextRec, CAST_ULONG(logData->msg[0].dt));
      if (logData->magic_start == LOGMAGIC_START)
      {
        unsigned long int item = logData->numRecs;
        ptr += sprintf(ptr, "Magic-Start found, len=%d", item);
        item = logData->nextRec;
        if (item < LOGMAX && CAST_ULONG(logData->msg[item].dt) == LOGMAGIC_END)
        {
          ptr += sprintf(ptr, "%s", ", Magic-End found");
          if (logData->Start.ts != 0) // !(!ts && numRecs) = ts || !numRecs
          {
            // falls Start.ts noch relativ ist (bei Absturz während NTP) muss dieser jetzt korrigiert werden
            portENTER_CRITICAL(&timeMux);
            if (logRealTS && logData->Start.ts < 1686249016)
            {
              // Start.ts bezieht sich auf die Sekunden seit dem Kaltstart, d.h. der echte Zeitpunkt ist TSreal = now - Kaltstart + TS
              ptr += sprintf(ptr, ", Start.ts %d=>", logData->Start.ts);
              logData->Start.ts += startTS;
              ptr += sprintf(ptr, "%d", logData->Start.ts);
            }
            portEXIT_CRITICAL(&timeMux);

            // Puffer-Start und Ende sind ok, nun die Ist-Werte prüfen
            Werte = logData->Start;
            applyLog(&Werte);

            if (memcmp(&Werte, &Ist, sizeof(Werte)) == 0)
            {
              ptr += sprintf(ptr, "%s", ", Data ok");
            }
            else
            {
              ptr += sprintf(ptr, "%s", ", Data inconsistent\nWerte:");
              for (int i = 0; i < sizeof(Werte); i++)
              {
                ptr += sprintf(ptr, " %02x", ((unsigned char *)&Werte)[i]);
              }
              ptr += sprintf(ptr, "%s", "\nIst:  ");
              for (int i = 0; i < sizeof(Ist); i++)
              {
                ptr += sprintf(ptr, " %02x", ((unsigned char *)&Ist)[i]);
              }
              ptr += '\n';
              ptr += 0;
              initLogData();
            }
          }
          else // if(logData->numRecs!=0 && logData->Start.ts!=0)
          {
            ptr += sprintf(ptr, "%s", ", Start.ts unset");
            initLogData();
          }
        }
        else
        {
          ptr += sprintf(ptr, "%s", ", Magic-End NOT found");
          initLogData();
        }
      }
      else
      {
        ptr += sprintf(ptr, "%s", "Magic-Start NOT found");
        #ifndef ARDUINO_LOLIN_S2_MINI
          printMemInfo();
        #endif
        initLogData();
      }
      gasLvlCnt = logData->gasLvl ? 50 : 0;
      ptr += sprintf(ptr, "magic_start = 0x%08x (0x%08x), numRecs = 0x%08x, nextRec = 0x%08x, msg[%d].dt = 0x%08x, Start.ts = 0x%08x\n",
                     logData->magic_start, LOGMAGIC_START, logData->numRecs, logData->nextRec, logData->nextRec, CAST_ULONG(logData->msg[logData->nextRec].dt), logData->Start.ts);
    }
    else initLogData();  // falls kein logData vorhanden, dann zumindest den Ist-Wert initialisieren
  }
  else
  {
    free(logData);
    logData = 0;
    ptr += sprintf(ptr, "free(logData)\n");
  }
#endif // DYNAMIC_LOGMEM

  ptr += sprintf(ptr, "\nESP.getFreeHeap() = %u\n", ESP.getFreeHeap());
  ptr += sprintf(ptr, "ESP.getMaxAllocHeap() = %u\n", ESP.getMaxAllocHeap());
  if (psramFound())
  {
    ptr += sprintf(ptr, "ESP.getFreePsram() = %u\n", ESP.getFreePsram());
    ptr += sprintf(ptr, "ESP.getMaxAllocPsram() = %u\n", ESP.getMaxAllocPsram());
  }

  request->send(200, "text/plain; charset=windows-1252", txt);
}

// ********************************** OTA-Update-Funktionen **********************************
// Serve: "/update"
const char* handleUpdateHTML =
  "<html><body>"
  "<form method='POST' action='/doUpdate' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>"
  "</body></html>";
void handleUpdate(AsyncWebServerRequest *request)
{
  request->send(200, "text/html; charset=windows-1252", handleUpdateHTML);
}

size_t content_len; // Länge des Update-Pakets

void printProgress(size_t prg, size_t sz)
{
  DBUG(Serial.printf("Progress: %d%%\r\n", (prg * 100) / content_len);)
  toggleLED(LED_BUILTIN);
}

void handleDoUpdate(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final)
{
  if (!index)
  {
    DBUG(Serial.printf("Update: %s\n", filename.c_str());)
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("spiffs") > -1) ?  U_PART : U_FLASH;
    //size_t fsSize = ((size_t) &_FS_end - (size_t) &_FS_start);
    //uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    content_len = request->contentLength();

  #ifdef ESP8266
    Update.runAsync(true);
    if (!Update.begin(content_len, cmd))
  #else
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
  #endif
    {
      Update.printError(Serial);
      return request->send(400, "text/plain", "OTA could not begin");
    }
  }

  // Write chunked data to the free sketch space
  if (len)
  {
    if (Update.write(data, len) != len)
    {
      Update.printError(Serial);
      return request->send(400, "text/plain", "OTA could not begin");
    }
  #ifdef ESP8266
    else
    {
      DBUG(Serial.printf("Progress: %d%%\n", (Update.progress() * 100) / Update.size());)
    }
  #endif
  }

  if (final) // if the final flag is set then this is the last frame of data
  {
    if (!Update.end(true))
    {
      Update.printError(Serial);
      return request->send(400, "text/plain", "Could not end OTA");
    }
    else
    {
      DBUG(Serial.println("Update complete");)
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Please wait while the device reboots");
      response->addHeader("Refresh", "5; url=/");
      response->addHeader("Location", "/");
      request->send(response);
      if (!Update.end(true))
      {
        Update.printError(Serial);
      }
      else
      {
        DBUG(Serial.println("Update complete");)
        DBUG(Serial.flush();)
        //ESP.restart();
        restartPending = true;
      }
    }
  }
}

//******************************* WebSocket-Callbacks ***************************************
// Verarbeitung eingehender Messages
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    DBUG(Serial.printf("WebSocketMsg: %s\n", (const char *)data);)
    const char *ptr = (const char *)data;
    while (*ptr)
    {
      if (strncmp(ptr, "Tag:", 4) == 0)
      {
        ptr += 4;
        Parameter.TempTag = atoi(ptr);
      }
      else if (strncmp(ptr, "Nacht:", 6) == 0)
      {
        ptr += 6;
        Parameter.TempNacht = atoi(ptr);
      }
      else if (strncmp(ptr, "Wasser:", 7) == 0)
      {
        ptr += 7;
        Parameter.TempWasser = atoi(ptr);
      }
      else if (strncmp(ptr, "Neigung:", 8) == 0)
      {
        ptr += 8;
        Parameter.Neigung = atoi(ptr);
      }
      else if (strncmp(ptr, "Niveau:", 7) == 0)
      {
        ptr += 7;
        Parameter.Niveau = atoi(ptr);
      }
      else if (strncmp(ptr, "Gas:", 4) == 0)
      {
        ptr += 4;
        if (logData)
        {
          signed long int dg = Ist.Gas - logData->Start.Gas;
          Ist.Gas = atoi(ptr);
          logData->Start.Gas = Ist.Gas - dg;
        }
        else
          Ist.Gas = atoi(ptr);
      }
      else if (strncmp(ptr, "WiFi:", 5) == 0)
      {
        ptr += 5;
        WiFi_switch = *ptr;
      }
      else if (strncmp(ptr, "LogLen:", 7) == 0)
      {
        ptr += 7;
        newLogLen = atoi(ptr);
      }
      while (*ptr != 0 && *ptr != '\r' && *ptr != '\n') ptr++;
      while (*ptr != 0 && (*ptr == '\r' || *ptr == '\n')) ptr++;
    }
  }
}

// Event-Callback
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
    case WS_EVT_CONNECT:
      DBUG(Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());)
      break;
    case WS_EVT_DISCONNECT:
      DBUG(Serial.printf("WebSocket client #%u disconnected\n", client->id());)
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

//********************************* WiFi-Funktionen *************************************
String wpspin2string(uint8_t a[])
{
  char wps_pin[9];
  for (int i = 0; i < 8; i++) {
    wps_pin[i] = a[i];
  }
  wps_pin[8] = '\0';
  return (String)wps_pin;
}

static const char *WiFi_Event_strings[] = {
  "WiFi ready", // 0 SYSTEM_EVENT_WIFI_READY
  "finish scanning AP", // 1 SYSTEM_EVENT_SCAN_DONE
  "station start", // 2 SYSTEM_EVENT_STA_START
  "station stop", // 3 SYSTEM_EVENT_STA_STOP
  "station connected to AP", // 4 SYSTEM_EVENT_STA_CONNECTED
  "station disconnected from AP", // 5 SYSTEM_EVENT_STA_DISCONNECTED
  "the auth mode of AP connected by ESP32 station changed", // 6 SYSTEM_EVENT_STA_AUTHMODE_CHANGE
  "station got IP from connected AP", // 7 SYSTEM_EVENT_STA_GOT_IP
  "station lost IP and the IP is reset to 0", // 8 SYSTEM_EVENT_STA_LOST_IP
  "station wps succeeds in enrollee mode", // 9 SYSTEM_EVENT_STA_WPS_ER_SUCCESS
  "station wps fails in enrollee mode", // 10 SYSTEM_EVENT_STA_WPS_ER_FAILED
  "station wps timeout in enrollee mode", // 11 SYSTEM_EVENT_STA_WPS_ER_TIMEOUT
  "station wps pin code in enrollee mode", // 12 SYSTEM_EVENT_STA_WPS_ER_PIN
  "station wps overlap in enrollee mode", // 13 SYSTEM_EVENT_STA_WPS_ER_PBC_OVERLAP
  "soft-AP start", // 14 SYSTEM_EVENT_AP_START
  "soft-AP stop", // 15 SYSTEM_EVENT_AP_STOP
  "a station connected to ESP32 soft-AP", // 16 SYSTEM_EVENT_AP_STACONNECTED
  "a station disconnected from ESP32 soft-AP", // 17 SYSTEM_EVENT_AP_STADISCONNECTED
  "soft-AP assign an IP to a connected station", // 18 SYSTEM_EVENT_AP_STAIPASSIGNED
  "Receive probe request packet in soft-AP interface", // 19 SYSTEM_EVENT_AP_PROBEREQRECVED
  "station or ap or ethernet interface v6IP addr is preferred", // 20 SYSTEM_EVENT_GOT_IP6
  "ethernet start", // 21 SYSTEM_EVENT_ETH_START
  "ethernet stop", // 22 SYSTEM_EVENT_ETH_STOP
  "ethernet phy link up", // 23 SYSTEM_EVENT_ETH_CONNECTED
  "ethernet phy link down", // 24 SYSTEM_EVENT_ETH_DISCONNECTED
  "ethernet got IP from connected AP"
}; // 25 SYSTEM_EVENT_ETH_GOT_IP

// events definiert in esp_event_legacy.h (c:\Users\Christian\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\)
//void WiFiEvent(WiFiEvent_t event, system_event_info_t info)
void WiFiEvent(arduino_event_id_t event, arduino_event_info_t info) // WiFiEventInfo_t info
{
  const char *WiFiStatusTxt[] = {"IDLE_STATUS", "NO_SSID_AVAIL", "SCAN_COMPLETED", "CONNECTED", "CONNECT_FAILED", "CONNECTION_LOST", "DISCONNECTED" };
  switch (event)
  {
    case SYSTEM_EVENT_STA_DISCONNECTED:
      DBUG(
        Serial.println("WiFiEvent 5: station disconnected, attempting reconnection");
        Serial.printf("Reason: %u - %s\n", info.wifi_sta_disconnected.reason, WiFi.disconnectReasonName((wifi_err_reason_t)info.wifi_sta_disconnected.reason));
        {
          unsigned short int sts = WiFi.status();
          Serial.printf("Status: %d - %s\n", sts, sts <= 6 ? WiFiStatusTxt[sts] : "unknown");
        }
      )
      //WiFi.reconnect();
      esp_wifi_connect(); // für den Reconnect
      toggleLED(LED_BUILTIN);  //Toggle LED Pin
      break;

  DBUG(
    case SYSTEM_EVENT_STA_START:
      Serial.println("WiFiEvent 2: Station Mode Started");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:   // info.wifi_sta_connected==system_event_sta_connected_t
      Serial.print("WiFiEvent 4: station connected to AP ");
      Serial.write(info.wifi_sta_connected.ssid, info.wifi_sta_connected.ssid_len);
      Serial.print(" auth=");
      Serial.write('0' + info.wifi_sta_connected.authmode);
      if (info.wifi_sta_connected.authmode < WIFI_AUTH_MAX)
      {
        Serial.print(" (");
        Serial.print(translateEncryptionType(info.wifi_sta_connected.authmode));
        Serial.println(')');
      }
      else Serial.write('\n');
      printSNTPservers();

      wifi_country_t country;
      if (esp_wifi_get_country(&country) == ESP_OK)
      {
        Serial.printf("get_country: >%-3.3s<, chan=%hhu-%hhu, txpwr=%hhd, policy=", country.cc, country.schan, country.schan + country.nchan, country.max_tx_power);
        if (country.policy == WIFI_COUNTRY_POLICY_AUTO) Serial.println("WIFI_COUNTRY_POLICY_AUTO");
        else Serial.println("WIFI_COUNTRY_POLICY_MANUAL");
      }
      wifi_bandwidth_t bw;
      if (esp_wifi_get_bandwidth(WIFI_IF_STA, &bw) == NULL)
      {
        Serial.print("bw=");
        if (bw == WIFI_BW_HT20) Serial.println("WIFI_BW_HT20");
        else if (bw == WIFI_BW_HT40) Serial.println("WIFI_BW_HT40");
        else Serial.println(bw);
      }
      break;

    case SYSTEM_EVENT_STA_GOT_IP:     // info.got_ip==system_event_sta_got_ip_t
      Serial.printf("WiFiEvent 7: got IP %08x\n", info.got_ip.ip_info.ip);
      printSNTPservers();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.print("WiFiEvent 12: WPS_PIN = ");
      Serial.println(wpspin2string(info.wps_er_pin.pin_code));
      break;
    default:
      if (event < SYSTEM_EVENT_MAX)
      {
        Serial.printf("WiFiEvent %d: %s\n", event, WiFi_Event_strings[event]);
      }
      else
      {
        Serial.print("WiFiEvnt: unknown ");
        Serial.println(event);
      }
      break;
  ) // end DBUG
  } // end switch(event)
}

// ************************************** UART-Funktionen ************************************
const char *uartErrorStrings[] = {
  "UART_NO_ERROR",
  "UART_BREAK_ERROR",
  "UART_BUFFER_FULL_ERROR",
  "UART_FIFO_OVF_ERROR",
  "UART_FRAME_ERROR",
  "UART_PARITY_ERROR"
};

void onRxError(hardwareSerial_error_t err)
{
  unsigned int e = err;
  if (e < 6)
  {
    log_v("RxErr [%d] %s.", err, uartErrorStrings[err]);
    //Serial.printf("RxErr [%d] %s.\n",err,uartErrorStrings[err]);
  }
  else
  {
    log_v("RxErr [%d].", err);
    //Serial.printf("RxErr [%d].\n",err);
  }
  // Wenn der Empfangspuffer zurückgesetzt wird, könnte bei Nachrichtenstau ein Telegramm verloren gehen
  //if(err == UART_BREAK_ERROR) trimatikMsgLen = 0;
}

void onRxDataB()
{
  size_t available = SerialB.available();
  //log_v("RxData: %d avail.",available);

  while (available)
  {
    if (trimatikMsgLen < MSGLEN_MAX)
    {
      trimatikMsg[trimatikMsgLen] = SerialB.read();
      trimatikMsgLen++;
    }
    else SerialB.read();
    SerB_cnt++;
    msgLastByteTS = millis();
    available = SerialB.available();
  }
}

void onRxDataA()
{
  size_t available = SerialA.available();
  //log_v("RxData: %d avail.",available);
  while (available)
  {
    if (trimatikMsgLen < MSGLEN_MAX)
    {
      trimatikMsg[trimatikMsgLen] = SerialA.read();
      trimatikMsgLen++;
    }
    else SerialA.read();
    SerA_cnt++;
    msgLastByteTS = millis();
    available = SerialA.available();
  }
}

//*********************************** NTP-Funktionen ******************************************
// vor ESP-IDF v5 ist time_t int32, was 2038 zu einem Überlauf führt
// ab ESP-IDF v5 ist time_t int64
// typedef int32_t time_t;
// struct timeval { time_t tv_sec;   /* seconds */  suseconds_t tv_usec;  /* and microseconds */ };
// time_is_set() wird aufgerufen, nachdem über SNTP ein Zeitstempel die Uhr gesetzt hat
void time_is_set(struct timeval *t) // the time received from the SNTP server
{
  unsigned long int runtime = millis();
  time_t now_ntp = time (nullptr);
  // alternativ in Millisekundengenauigkeit:
  /*
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t now_ms = ((int64_t)tv_now.tv_sec) * 1000L + (int64_t)(tv_now.tv_usec/1000);
    Serial.printf("time()=%d, gettimeofday()=%f\n",now_ntp,now_ms/1000.0f);
  */

  ntpSyncCount++;
  if (t != nullptr) ntpSyncTS = t->tv_sec; else ntpSyncTS = 0;

  //DBUG(printSNTPservers();)
  /*DBUG(
    Serial.print("------------------ settimeofday() was called ------------------");
    Serial.print("millis()="); Serial.println(runtime);
    Serial.print("local asctime: "); Serial.print(asctime (localtime (&now))); // print formated local time
    if (t != nullptr)
    {
      Serial.print("local asctime arg: "); Serial.print(asctime (localtime (&t->tv_sec))); // print formated local time
    }
    Serial.printf("time()=%d\n", now);
    )*/

  // wenn das erste mal ein gültiger Zeitstempel vorliegt müssen die TS im Puffer korrigiert werden:
  // in setup() wird in startTS die resetfeste Uhrzeit abgelegt, was die Sekunden seit dem Kaltstart angibt.
  // millis() enthält die Zeit seit dem Reset/Warmstart.
  // D.h. startTS - millis()/1000 ist der Zeitpunkt des Resets/Warmstarts.
  // Steht nun in 'now_ntp' eine absolute Zeitangabe zur Verfügung, so ist now_ntp - millis()/1000 der Zeitpunkt des Resets/Warmstarts,
  // und now_ntp - millis()/1000 - startTS der absolute Zeitpunkt des Kaltstarts.
  // Wenn Start.ts keine absolute Zeitangabe enthält, so ist es bezogen auf den Kaltstart.
  // D.h. auf Start.ts muss der Kaltstartzeitpunkt addiert werden.
  if (!logRealTS)
  {
    // wenn logRealTS==false war, dann ist startTS relativ und kann nun korrigiert werden
    startTS = now_ntp - startTS - (runtime / 1000);
    // wenn eine Aufzeichnung vorhanden ist, TimeStamps korrigieren
    portENTER_CRITICAL(&timeMux);
    logRealTS = true;
    if (logData && logData->Start.ts < 1686249016)
    {
      // Start.ts bezieht sich auf die Sekunden seit dem Kaltstart, d.h. der echte Zeitpunkt ist TSreal = now_ntp - Kaltstart + TS
      //log_v("Start.ts %d=>%d", logData->Start.ts, logData->Start.ts + startTS);
      logData->Start.ts += startTS;
    }
    if (Ist.ts < 1686249016)
    {
      // Ist.ts bezieht sich auf die Sekunden seit dem Kaltstart, d.h. der echte Zeitpunkt ist TSreal = now_ntp - Kaltstart + TS
      //log_v("Ist.ts %d=>%d", Ist.ts, Ist.ts + startTS);
      Ist.ts += startTS;
    }
    if (now < 1686249016)
    {
      now += startTS;
    }
    portEXIT_CRITICAL(&timeMux);
  }
}

// the setup function runs once when you press reset or power the board
void setup()
{
  logData = 0;
  newdata = false;
  logRealTS = false;  // noch keine Uhrzeit vorhanden
  msgLastByteTS = 0xffffffff;

  trimatikMsgLen = 0;
  Parameter.resetCount++;

  // für Debug-Ausgabe
  // beim ESP32-S2 wird (bei "USB CDC on boot") die UART0 auf "Serial0" gelegt. "Serial" ist der USB-VCP. Ansonsten wird "Serial"==SerialA für den Debug-Output genutzt, wenn NODEBUG nicht gesetzt ist
  #ifdef ARDUINO_LOLIN_S2_MINI
    #if ARDUINO_USB_CDC_ON_BOOT   // nur wenn der VCP vorhanden ist
      Serial.begin(115200);
    #endif
  #else // beim ESP32 ist Serial die UART0 und unabhängig von SerialA und SerialB
    Serial.begin(115200);
  #endif
  //Serial.setDebugOutput();
  //Serial.setHwFlowCtrlMode(HW_FLOWCTRL_DISABLE);
  //Serial.setDebugOutput(true);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(0, INPUT_PULLUP);
  // hier kann der Start durch Drücken des IO0-Tasters aufgehalten werden
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(LED_BUILTIN, 1);
    delay(30);
    while (digitalRead(0) == 0);
    digitalWrite(LED_BUILTIN, 0);
    delay(50);
  }

  // Trimatik Channel-B: 1200 Baud, 8E1
  // die TimeOuts funktionieren nur, wenn Serial1.begin(onlyOnTimeout==true)
  // bei v2.0.14 kommen nur über Ser1 Daten
  SerialB.setTimeout(1000);
  SerialB.setRxTimeout(10);  // in Byte-Zeiteinheiten : T[s] = N*11Bit/2400baud => 1s = 109 * 11 Bit/1200baud
  SerialB.onReceiveError(onRxError);
  SerialB.onReceive(onRxDataB, false);  // onlyOnTimeout
  SerialB.begin(BAUDRATE, SERIAL_8E1, CHANNEL_B, TXDUMMY_B);    // Serial1 benutzt GPIO18=RxD, GPIO17=TxD bzw.  GPIO14=RxD, GPIO15=TxD
  SerialA.setTimeout(1000);
  SerialA.setRxTimeout(10);  // in Byte-Zeiteinheiten : T[s] = N*11Bit/2400baud => 1s = 109 * 11 Bit/1200baud
  SerialA.onReceiveError(onRxError);
  SerialA.onReceive(onRxDataA, false);  // onlyOnTimeout
  SerialA.begin(BAUDRATE, SERIAL_8E1, CHANNEL_A, TXDUMMY_A);    // Serial0 benutzt GPIO16=RxD, GPIO13=TxD

  /*while (digitalRead(0) == 1)
    {
    delay(1000);
    Serial.println("Serial");
    Serial0.println("Serial0");
    Serial1.println("Serial1");
    digitalWrite(LED_BUILTIN, 1);
    delay(30);
    digitalWrite(LED_BUILTIN, 0);
    }
    while (digitalRead(0) == 0);*/

  DBUG(
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    delay(1000);
    Serial.println("\nbuilt " __DATE__ " " __TIME__);
    Serial.printf("Debug level: %d\n", CORE_DEBUG_LEVEL);
    log_e("error");
    log_w("warning");
    log_i("info");
    log_d("debug");
    log_v("verbose");

    // CPU Info
    Serial.printf("getXtalFrequencyMhz()=%d MHz\n", getXtalFrequencyMhz());
    Serial.printf("getCpuFrequencyMhz()=%d MHz\n", getCpuFrequencyMhz());
    Serial.printf("getApbFrequency()=%d Hz\n", getApbFrequency());
    
    //esp_chip_info();
    uint32_t chipId = 0;
    for (int i = 0; i < 17; i = i + 8)
    {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    int numCores = ESP.getChipCores();
    Serial.printf("This chip has %d cores\n", numCores);
    Serial.printf("Chip ID: 0x%06X\n", chipId);
    
    // Reset-Reason output
    for (int c = 0; c < numCores; c++)
    {
      Serial.printf("CPU%d reset reason: ", c);
      RESET_REASON reason = rtc_get_reset_reason(c);
      Serial.print(reset_reason(reason));
      Serial.print(' ');
      Serial.println(verbose_reset_reason(reason));
    }
    Serial.printf("Chip-Temperatur: %.1f°C\n", temperatureRead());
    
    if (psramFound())
    {
      Serial.println("========================================");
      //Serial.printf("PSRAM total size     : %u \n", esp_spiram_get_size());
      //Serial.println("----------------------------------------");
      Serial.printf("PSRAM size    : %u \n", ESP.getPsramSize());
      Serial.printf("PSRAM free    : %u \n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
      Serial.printf("PSRAM largest : %u \n", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    }
    Serial.println("========================================");
    Serial.printf("Internal RAM  size   : %u \n", ESP.getHeapSize());
    Serial.printf("Internal RAM  free   : %u \n", heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
    Serial.printf("Internal RAM  largest: %u \n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
    Serial.println("========================================");
  ) // end DBUG

  enableLoopWDT();

  pinMode(CHANNEL_A, INPUT_PULLUP);
  gpio_set_pull_mode((gpio_num_t)CHANNEL_B, GPIO_PULLUP_ONLY);  //pinMode(CHANNEL_B, INPUT_PULLUP);
  pinMode(GAS_COUNTER, INPUT_PULLUP);
  /*
    attachInterrupt(digitalPinToInterrupt(GAS_COUNTER), GasCounterChgISR, FALLING);   // Interrupt bei x.x11
  */

  // Bei einem Soft-Reset läuft die RTC weiter, d.h. time() wird nicht zurückgesetzt
  // time_t now, startTS (signed long)
  // time_t	   _EXFUN(time,     (time_t *_timer));
  // unsigned long millis() = esp_timer_get_time() / 1000ULL;
  // int64_t esp_timer_get_time();
  now = time(nullptr);
  //unsigned long runtime = millis() / 1000;
  time_t runtime = (time_t)(esp_timer_get_time() / 1000000ULL);
  startTS = now - runtime;    // startTS enthält die Kaltstartzeit in Sekunden, für den Fall, dass time() keine exakte Zeit enthält
  DBUG(Serial.printf("startTS = now - runtime = %d - %d = %d\n", now, runtime, startTS);)
  // wenn time() gültig ist, wird davon ausgegangen, dass keine relative Zeitangabe im Puffer existiert
  if (now > 1686249016)
  {
    logRealTS = true;
    // Ist.ts ist nach einem Kaltstart undefiniert - durchaus !=0 - wird dann in allocLogMem() initialisiert.
    // Nach einem Warmstart, ohne vorherige Daten, ist Ist.ts == 0.
    // Nach einem Warmstart, mit Daten aber Absturz während NTP ist Ist.ts klein - da wirkt die startTS-Korrektur
    // Die gleiche Korrektur an logData->Start.ts wird in allocLogMem() durchgeführt.
    if (Ist.ts != 0 && Ist.ts < 1686249016)
    {
      // Ist.ts bezieht sich auf die Sekunden seit dem Kaltstart, d.h. der echte Zeitpunkt ist TSreal = now - Kaltstart + TS
      DBUG(Serial.printf("Ist.ts %d=>", Ist.ts);)
      Ist.ts += startTS;
      DBUG(Serial.println(Ist.ts);)
    }
  }

  allocLogMem();  // benötigt startTS in der log-Prüfung zur Korrektur von logData->Start.ts

  // falls mal ein Timer-Interrupt benötigt wird
  //timer = timerBegin(0, 80, true);
  //timerAttachInterrupt(timer, &onTimerISR, true);

  // NTP-Setup: wenn vor DHCP, dann wird der NTP erfragt
  DBUG(
    Serial.println("initial time:");
    Serial.print("millis()/1000=");  Serial.println(runtime);
    Serial.print("local asctime: "); Serial.print(asctime (localtime (&now))); // print formated local time
    Serial.print("time()="); Serial.println(now);
    //Serial.printf("time()=%d\n",now);

    /*struct tm timeinfo;
      Serial.print("getLocalTime()... ");
      if(!getLocalTime(&timeinfo))
      {
      Serial.println("Failed to obtain time");
      }
      else
      {
      Serial.print("&timeinfo... ");
      Serial.println(&timeinfo, "%d %b %Y %H:%M:%S %z");
      }*/
    //printSNTPservers();
    Serial.print("initial SNTP-Sync-Interval: ");
    Serial.println(sntp_get_sync_interval());
  ) // end DBUG
  sntp_set_time_sync_notification_cb( time_is_set );
  sntp_set_sync_interval(24 * 60 * 60 * 1000); // Sync-Intervall in [ms]
  sntp_servermode_dhcp(1);
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov"); // die Server werden verwendet, wenn über DHCP keine mitgeteilt worden sind
  DBUG(
    Serial.println("NTP time sync ready.");
    //printSNTPservers();
  )

  esp_err_t err;
  //err = esp_wifi_set_ps(WIFI_PS_NONE);
  //Serial.printf("esp_wifi_set_ps(WIFI_PS_NONE)=%d\n",type,err);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.onEvent(WiFiEvent);

  //log_v("WiFi.getMode() = %d", WiFi.getMode());
  log_v("WiFi.mode(WIFI_STA)...");
  WiFi.mode(WIFI_STA);
  log_v("WiFi.getMode() = %d", WiFi.getMode());
  WiFi.setAutoReconnect(false); // der AutoReconnect wird im Event-Callback durchgeführt

  WiFi.setHostname(host);   // keine Ahnung wo der String verwendet wird...

  err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  DBUG(Serial.printf("esp_wifi_set_bandwidth(WIFI_BW_HT20)=>%d\n", err);)
  DBUG(
    uint8_t protocols;
    err = esp_wifi_get_protocol(WIFI_IF_STA, &protocols);
    if (err == ESP_OK)
    {
      Serial.print("protocols:");
      if (protocols & WIFI_PROTOCOL_11B) Serial.print(" 11B");
      if (protocols & WIFI_PROTOCOL_11G) Serial.print(" 11G");
      if (protocols & WIFI_PROTOCOL_11N) Serial.print(" 11N");
      if (protocols & WIFI_PROTOCOL_LR) Serial.print(" LR");
    }
    else
    {
      Serial.print("esp_wifi_get_protocol() failed: ");
      Serial.println(err);
    }
    wifi_bandwidth_t bw;
    err = esp_wifi_get_bandwidth(WIFI_IF_STA, &bw);
    if (err == ESP_OK)
    {
      Serial.print(", bw=");
      if (bw == WIFI_BW_HT20) Serial.println("WIFI_BW_HT20");
      else if (bw == WIFI_BW_HT40) Serial.println("WIFI_BW_HT40");
      else Serial.println(bw);
    }
    else
    {
      Serial.print("esp_wifi_get_bandwidth() failed: ");
      Serial.println(err);
    }
    wifi_ps_type_t type;
    err = esp_wifi_get_ps(&type);
    Serial.printf("esp_wifi_get_ps(%d)=%d\n", type, err);
    //err = esp_wifi_set_ps(WIFI_PS_NONE);
    //Serial.printf("esp_wifi_set_ps(WIFI_PS_NONE)=%d\n",type,err);

    wifi_config_t current_conf;
    err = esp_wifi_get_config(WIFI_IF_STA, &current_conf);
    if (err == ESP_OK)
    {
      Serial.printf("conf: SSID='%.32s', PSK='%.64s'\n", current_conf.sta.ssid, current_conf.sta.password);
    }
    else
    {
      Serial.println("esp_wifi_get_config() failed.");
    }
  ) // end DBUG

  log_v("WiFi.begin()...");
  DBUG(Serial.println("WiFi.begin()...");)

  WiFi.begin(ssid, password);

  log_v("WiFi.getMode() = %d", WiFi.getMode());
  tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, host);  // hier schon eher

  log_v("MDNS.begin...");
  MDNS.begin(host);

  //********************************* Setup Server ********************/
  log_v("Setup Webserver...");
  // Webseiten
  // Handle Web Server
  //httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {request->redirect("/update");});
  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveIndexPage(request);
  });
  httpServer.on("/msglist.txt", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveMsgList(request);
  });
  httpServer.on("/messwerte.json", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveMesswerteJSON(request);
  });
  httpServer.on("/messwerte.csv", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveMesswerteCSV(request);
  });
  httpServer.on("/messwerte.bin", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveMesswerteBIN(request);
  });
  httpServer.on("/param.json", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveParamJSON(request);
  });
  httpServer.on("/wifiscan.txt", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveWiFiScan(request);
  });
  httpServer.on("/logreport.txt", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveLogReport(request);
  });
  httpServer.on("/info.txt", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveInfoTxt(request);
  });
  httpServer.on("/psram.txt", HTTP_GET, [](AsyncWebServerRequest * request) {
    servePSRAMTxt(request);
  });
  httpServer.on("/ram.txt", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveRAMTxt(request);
  });
  httpServer.on("/logmem.txt", HTTP_GET, [](AsyncWebServerRequest * request) {
    serveLogTxt(request);
  });
  httpServer.on("/update", HTTP_GET, [](AsyncWebServerRequest * request) {
    handleUpdate(request);
  });
  httpServer.on("/doUpdate", HTTP_POST,
                [](AsyncWebServerRequest * request)
                {
                  bool shouldReboot = !Update.hasError();
                  // the request handler is triggered after the upload has finished...
                  // create the response, add header, and send response
                  AsyncWebServerResponse *response = request->beginResponse(shouldReboot ? 200 : 500, "text/plain", shouldReboot ? "OK" : "FAIL");
                  response->addHeader("Connection", "close");
                  response->addHeader("Access-Control-Allow-Origin", "*");
                  request->send(response);
                  yield();
                  delay(1000);
                  yield();
                  //ESP.restart();
                  restartPending = true;
                },
                [](AsyncWebServerRequest * request, const String & filename, size_t index, uint8_t *data, size_t len, bool final)
                {
                  //Upload handler chunks in data
                  handleDoUpdate(request, filename, index, data, len, final);
                });

  Update.onProgress(printProgress);
  httpServer.onNotFound([](AsyncWebServerRequest * request)
                        {
                          request->send(404);
                        });

  ws.onEvent(onEvent);
  httpServer.addHandler(&ws);
  log_v("Start Webserver...");
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);

  log_v("setup() done.");
}

// the loop function runs over and over again forever
void loop()
{
  if (restartPending) ESP.restart();

  static unsigned short int softReset = 0;
  // RESET_BY_LOOP
  if (softReset < 20)
  {
    if (digitalRead(0) == 0)
    {
      softReset++;
      toggleLED(LED_BUILTIN);
      DBUG(Serial.printf("softReset=%d\n", softReset);)
    }
    else if (softReset)
    {
      softReset = 0;
      digitalWrite(LED_BUILTIN, 0);
      DBUG(Serial.println("softReset=0");)
    }
  }
  else ESP.restart();

  //log_v("Enter loop()...");
  //    Serial.println(__LINE__);    delay(500);
#ifdef ESP8266
  MDNS.update();
#endif

  // zentraler Zeitstempel für alle kommenden dt's
  now = time(nullptr);

  // Auswertung des seriellen Datenpakets nach RX_TIMEOUT
  #define RX_TIMEOUT 1000
  if (msgLastByteTS < (millis() - RX_TIMEOUT))
  {
    DBUG(
      Serial.print("RxTimeout: ");
      if (trimatikMsgLen > 0)
      {
        for (int i = 0; i < trimatikMsgLen; i++) Serial.printf("%02x ", trimatikMsg[i]);
        Serial.println("");
      }
    )

    if (trimatikMsgLen > 1)
    {
      char buf[200 + MSGLEN_MAX * 3];
      // Werte aus Datenpaket verarbeiten
      processMsg(buf, trimatikMsg, trimatikMsgLen);
      // Sende Info an WebSocket
      ws.textAll(buf);

      // kann verwendet werden, wenn auf geänderte Daten reagiert werden muss
      if (logData && newdata)
      {
        newdata = false;
      }
    }

    SerA_cnt = 0;
    SerB_cnt = 0;
    trimatikMsgLen = 0;
    msgLastByteTS = 0xffffffff;
  }

  if (trimatikMsgLen) toggleLED(LED_BUILTIN);
  delay(100);

  // Einlesen des Gaszähler-Eingangs mit Entprellung
  if (digitalRead(GAS_COUNTER) == HIGH)
  {
    if (gasLvlCnt < 50) gasLvlCnt++; // wenn 50x0,1s=5s HIGH war, dann wird es als HIGH interpretiert
    else if (logData) logData->gasLvl = HIGH;
  }
  else
  {
    if (gasLvlCnt > 5)  gasLvlCnt -= 5; // wenn 10x0,1s=1s LOW war, dann wird es als LOW interpretiert
    else
    {
      gasLvlCnt = 0;
      if (logData)
      {
        if (logData->gasLvl == HIGH)
        {
          logData->gasLvl = LOW;
          Ist.Gas++;
          newdata = true;
          const unsigned char gasValue[3] = {0, 1, 1};
          logValue(gasValue);
        }
      }
    }
  }

  // wenn der WiFi-Mode geändert werden soll
  if (WiFi_switch)
  {
    esp_err_t err;
    uint8_t protocols;
    err = esp_wifi_get_protocol(WIFI_IF_STA, &protocols);
    if (err == ESP_OK)
    {
      if (WiFi_switch == 'n')
      { // n bei Bedarf erlauben
        if ((protocols & WIFI_PROTOCOL_11N) == 0)
        {
          DBUG(Serial.println("Enable 802.11n");)

          //err = esp_wifi_stop();
          err = esp_wifi_disconnect();
          if (err == ESP_OK)
          {
            err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N); // muss nach esp_wifi_init() und vor esp_wifi_start() passieren (also müsste in WiFi.mode() passieren)
            if (err != ESP_OK)
            {
              DBUG( Serial.print("esp_wifi_set_protocol() failed: "); Serial.println(err); )
            }
            err = esp_wifi_connect();
            if (err != ESP_OK)
            {
              DBUG( Serial.print("esp_wifi_connect() failed: "); Serial.println(err); )
            }
          }
          else
          {
            DBUG( Serial.print("esp_wifi_disconnect() failed: "); Serial.println(err); )
            //DBUG( Serial.print("esp_wifi_stop() failed: "); Serial.println(err); )
          }
        }
      }
      else if (WiFi_switch == 'g')
      { // n bei Bedarf sperren, nur g und b erlauben
        if (protocols & WIFI_PROTOCOL_11N)
        {
          DBUG(Serial.println("Disable 802.11n");)
          //err = esp_wifi_stop();
          err = esp_wifi_disconnect();
          if (err == ESP_OK)
          {
            err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G); // muss nach esp_wifi_init() und vor esp_wifi_start() passieren (also müsste in WiFi.mode() passieren)
            if (err != ESP_OK)
            {
              DBUG( Serial.print("esp_wifi_set_protocol() failed: "); Serial.println(err); )
            }
            err = esp_wifi_connect();
            if (err != ESP_OK)
            {
              DBUG( Serial.print("esp_wifi_connect() failed: "); Serial.println(err); )
            }
          }
          else
          {
            DBUG( Serial.print("esp_wifi_disconnect() failed: "); Serial.println(err); )
            //DBUG( Serial.print("esp_wifi_stop() failed: "); Serial.println(err); )
          }
        }
      }
      // alles verarbeitet, WiFi_switch zurücksetzen
      WiFi_switch = 0;
    } // end if esp_wifi_get_protocol()
  } // end if(WiFi_switch)

  // wenn der Ringpuffer gekürzt werden soll
  if (newLogLen)
  {
    if (newLogLen < LOGMAX)
    {
      // Pufferreduktion von LOGMAX auf newLogLen Datensätze, wenn mehr als newLogLen Records vorhanden sind
      if (logData->numRecs > newLogLen) // z.B. LOGMAX=4, numRecs=4, nextRec=0, newLogLen=2
      {
        unsigned short int src;
        unsigned short int toGo;
        // als erstes die zu streichenden Datensätze auf den Start anwenden
        if (logData->numRecs < LOGMAX) // der Ring ist noch nicht aktiv, der Start ist 0
        {
          src = 0;
          toGo = logData->numRecs - newLogLen + 1;
        }
        else
        {
          src = logData->nextRec; // nextRec zeigt aufs MagicEnd
          // src muss auf den nächsten = ältesten Record verschoben werden
          src++;
          if (src == LOGMAX) src = 0;
          toGo = LOGMAX - newLogLen;
        } // z.B. src = 1, toGo=2
        // es sollen newLogLen Records übrig bleiben, d.h. es müssen numRecs-newLogLen Records angewendet werden
        while (toGo != 0)
        {
          applyLogItem(&logData->Start, src);
          src++;
          if (src == LOGMAX) src = 0;
          toGo--;
        } // z.B. src=3
        //toGo = 0;
        //src = toGo; also der Record, der als ältester erhalten bleibt und nach [0] kopiert werden muss; Ende wird nextRec mit dem MagicEnd sein
        if (src < logData->nextRec) // wenn es beim Kopieren zu keinem Ringüberlauf kommt, dann wird vorwärts kopiert
        { // z.B. nextRec=3, src=2 : toGo=2 [2]=>[0], [3]=>[1]
          // z.B. nextRec=2, src=1 : toGo=2 [1]=>[0], [2]=>[1]
          unsigned short int dst = 0;
          toGo = logData->nextRec - src + 1;
          while (toGo)
          {
            logData->msg[dst] = logData->msg[src];
            dst++;
            src++;
            toGo--;
          }
        }
        else  // das Ende der Quellbereichs (nextRec) liegt vor dem Anfang (src), d.h. es findet ein Ringüberlauf statt: es wird rückwärts kopiert
        { // z.B. nextRec=0, src=3 : src=0, dst=4+0-3=1, toGo=2
          unsigned short int dst = LOGMAX + logData->nextRec - src; // beginnend vom Ende des Zielbereichs (=Länge der Quelldaten)
          src = logData->nextRec;     // beginnend vom Ende des Quellbereichs
          toGo = dst + 1; // sind insgesamt dst+1 Bytes
          while (toGo)
          {
            logData->msg[dst] = logData->msg[src];
            if (src == 0) src = LOGMAX - 1; else src--;
            dst--;
            toGo--;
          }
        } // end if Copy-Dir
        logData->numRecs = newLogLen;
        logData->nextRec = newLogLen;
      } // end if(logData->numRecs > newLogLen)
    }
    newLogLen = 0;
  }
}

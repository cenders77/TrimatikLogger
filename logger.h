#ifndef TRIMATIK_LOGGER_H
#define TRIMATIK_LOGGER_H

#define xstr(s) str(s)
#define str(s) #s

#define CAST_ULONG(x) *(unsigned long int *)(&(x))

// die Struktur trimatikWerte enthält alle Werte zu einem bestimmten Zeitpunkt ts
// wird für Start- und Ist-Datensatz benutzt
typedef struct
{
  time_t ts;
  unsigned long int Gas;
  unsigned char Relais;   // 0x25 Ausgänge
  unsigned char Status;   // 0x2f ???, alle 1,7s
  unsigned char Aussen;   // 0x31 AussentempX2
  unsigned char Wasser;   // 0x35 WarmwasserX2
  unsigned char Kessel;   // 0x3a Kesseltemp1X2, alle 1,7s
  unsigned char Programm; // 0x45 von Schaltuhr: aktive Kanäle, alle 1,7s
  unsigned char Tag;      // 0x7f von Schaltuhr: Tageswechsel, alle 1,7s
  unsigned char x26;      // 0x26
  unsigned char Vorlauf;  // 0x32 VorlauftempX2
  unsigned char Kessel2;  // 0x33 Kesseltemp2X2
  unsigned char x34;      // 0x34
} trimatikWerte;

// trimaticRecord beschreibt das Element, aus dem der Ringpuffer besteht
typedef struct
{
  signed short int dt;    // 32767s = 9h6min
  unsigned char id;       // WertID
  unsigned char value;    // Wert; bei Gas: Inkrement
} trimatikRecord;

// trimatikLog beschreibt die Struktur des gesamten Logger-Datensatzes
typedef struct
{
  unsigned long int magic_start;
  unsigned short int numRecs, nextRec;
  bool gasLvl;
  trimatikWerte Start;    // Datensatz mit den Start-Werten
  trimatikRecord msg[1];  // Records mit den aufgezeichneten Werten
} trimatikLog;

// trimatikParameter beschreibt die Struktur, die die Einstellungen der Regelung enthält
typedef struct
{
  unsigned short int resetCount;
  signed char TempTag;
  signed char TempNacht;
  signed char TempWasser; // Standard 44
  unsigned char Neigung;  // *10, Standard 1,2
  signed char Niveau;
} trimatikParameter;

// logCtx beschreibt den Kontext für die Verarbeitung eines Requests
typedef struct
{
  unsigned short int mIdx;  // mIdx gibt das nächste zu übertragende Element im Ringpuffer an
  unsigned short int mToGo; // mToGo gibt die Anzahl noch zu übertragender Elemente an
  trimatikWerte Werte;      // enthält den bisherigen Zustand
} logCtx;

// MW2CSV_ctx beschreibt den Kontext für die Verarbeitung eines Requests für CSV
typedef struct
{
  logCtx log;            // enthält mIdx, mToGo und Werte-Satz
  bool decimalComma;        // wie der Dezimalpunkt dargestellt wird: als ',' oder '.'
  unsigned short partIdx;   // Index innerhalb des 'part'-Puffers
  unsigned short partLen;   // Länge des 'part'-Puffers
  char part[80]; // hier muss eine ganze Zeile reinpassen: CSV=80, JSON=72
} MW2CSV_ctx;

// MW2JSON_ctx beschreibt den Kontext für die Verarbeitung eines Requests für JSON
typedef struct
{
  logCtx log;            // enthält mIdx, mToGo und Werte-Satz
  unsigned short partIdx;   // Index innerhalb des 'part'-Puffers
  unsigned short partLen;   // Länge des 'part'-Puffers
  char part[80]; // hier muss eine ganze Zeile reinpassen: CSV=80, JSON=72
} MW2JSON_ctx;


#define LOGMAGIC_START  0x5452494D  // "TRIM"
#define LOGMAGIC_END    0x4154494B  // "ATIK"
#define LOGMEMSIZE (sizeof(trimatikLog) + (LOGMAX-1)*sizeof(trimatikRecord))

#endif

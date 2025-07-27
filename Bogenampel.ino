#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <esp_now.h> // ESP-NOW Bibliothek hinzufügen
#include <EEPROM.h>  // EEPROM Bibliothek hinzufügen
#include <U8g2lib.h> // U8g2 Bibliothek hinzufügen
#include <Update.h> // Hinzufügen für Web-Upload OTA
#include <esp_ota_ops.h> // Hinzufügen für UPDATE_SIZE_UNKNOWN
#include <ArduinoJson.h> // Stelle sicher, dass dies am Anfang der Datei steht
#include <vector>  // NEU: Für die Liste der zu löschenden Peers
#include <array>   // NEU: Für die MAC-Adressen in der Liste
#include <Button2.h> // Button2 Bibliothek hinzufügen
#include <ESP32Encoder.h> // NEU: Bibliothek für den KY-040 Drehgeber
#include <WiFiManager.h> // Für die dynamische WLAN-Konfiguration

// Definition des Ampelzustands
enum State { ROT, GELB_PREP, GRUEN, GELB_REST };

// --- NEUE Struktur für die Kopplung ---
enum MessageType : uint8_t {
  STATE_UPDATE,
  PAIRING_REQUEST
};

struct GenericMessage {
  MessageType type;
};

struct PairingMessage {
  MessageType type;
  uint8_t macAddr[6];
};

// Definition von Ton-Typen für die NRF Kommunikation und Steuerung
enum ToneType {
  TONE_NONE,
  TONE_GO_TO_LINE,    // Zweimaliges Pfeifen: Zur Schießlinie gehen (Regel 6.7.2.1, 1. Punkt)
  TONE_SHOOT_START,   // Einmaliges Pfeifen: Schießbeginn (Regel 6.7.2.1, 2. und 5. Punkt)
  TONE_GROUP_CHANGE,  // Zweimaliges Pfeifen: Schießzeit beendet, Gruppenwechsel (Regel 6.7.2.1, 3. Punkt)
  TONE_END_OF_END,    // Dreimaliges Pfeifen: Schießzeit beendet, Trefferaufnahme (Regel 6.7.2.1, 6. Punkt)
  TONE_EMERGENCY      // Reihe aufeinanderfolgender Pfiffe: Gefahr (Regel 6.7.2.1, 7. Punkt)
};

// Passe die AmpelState Nachricht an, um den Typ zu enthalten
struct AmpelStateMessage {
  MessageType type;
  State state;
  int abCdPhase;
  ToneType toneType;
  bool isStartingGroupAB;
  uint8_t shootingTimeIndex;
  uint8_t targetModeIndex;
};

// --- NEU: SerialAndWebLogger Klasse ---
class SerialAndWebLogger : public Print {
private:
  HardwareSerial& _realSerial;
  std::vector<String> _logBuffer;
  String _currentLineBuffer;
  const size_t _maxLogLines = 50; // Maximale Anzahl Zeilen im Web-Log
  bool _webLogEnabled = false;

public:
  SerialAndWebLogger(HardwareSerial& serial) : _realSerial(serial) {}

  virtual size_t write(uint8_t c) {
    if (_webLogEnabled) {
      _currentLineBuffer += (char)c;
      if (c == '\n') {
        if (_logBuffer.size() >= _maxLogLines) {
          _logBuffer.erase(_logBuffer.begin());
        }
        _logBuffer.push_back(_currentLineBuffer);
        _currentLineBuffer = "";
      }
    }
    return _realSerial.write(c);
  }

  virtual size_t write(const uint8_t *buffer, size_t size) {
    if (_webLogEnabled) {
      for (size_t i = 0; i < size; i++) {
        write(buffer[i]); // Ruft die einzelne write(uint8_t) Methode auf
      }
    }
    return _realSerial.write(buffer, size);
  }

  void enableWebLog(bool enable) { _webLogEnabled = enable; }
  bool isWebLogEnabled() { return _webLogEnabled; }
  void clearWebLog() { _logBuffer.clear(); _currentLineBuffer = ""; }
  String getWebLog() {
    String logText = "";
    for (const String& line : _logBuffer) { logText += line; }
    return logText;
  }
  // Übernahme der flush() Methode von HardwareSerial, falls benötigt
  void flush() { _realSerial.flush(); }
};

// Definition der LED-Pins
const int redLedPin = 2;
const int yellowLedPin = 4;
const int greenLedPin = 16;
const int abLedPin = 17; // Zusätzliche LED für AB Gruppe
const int cdLedPin = 5;  // Zusätzliche LED für CD Gruppe

// Definition des Piezo-Summer Pins
const int piezoPin = 18;

// NEU: Definition der KY-040 Drehgeber-Pins
const int ENCODER_CLK_PIN = 19; // CLK-Pin des Encoders
const int ENCODER_DT_PIN = 27;  // DT-Pin des Encoders
const int ENCODER_SW_PIN = 32;  // SW-Pin (Taster) des Encoders
// NEU: Variable zum Speichern des letzten Encoder-Zustands
long oldEncoderValue = 0;

// Definition der zusätzlichen Taster-Pins
const int startStopButtonPin = 25; // Pin für Start/Stop Taster
const int emergencyButtonPin = 26; // Pin für Notknopf Taster

AmpelStateMessage currentStateData;

// NEU: Struktur für Keep-Alive Nachrichten
enum KeepAliveType : uint8_t { PING = 0, PONG = 1 };
struct KeepAliveMsg {
  KeepAliveType type;
};


// --- ESP-NOW spezifische Variablen ---
#define MAX_SLAVES 5 // Maximale Anzahl von Slaves definieren

// Array für die MAC-Adressen der Slaves.
// Wird aus EEPROM geladen oder initialisiert.
uint8_t slaveMacs[MAX_SLAVES][6] = {
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // Beispiel MAC Slave 1 (Initial Broadcast/Leer)
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // Beispiel MAC Slave 2 (Initial Broadcast/Leer)
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // Beispiel MAC Slave 3
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // Beispiel MAC Slave 4
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // Beispiel MAC Slave 5
};
int numRegisteredSlaves = 0; // Zähler für tatsächlich registrierte Slaves (basierend auf nicht-leeren Einträgen)

esp_now_peer_info_t peerInfo;

// --- EEPROM Konfiguration ---
// Größe: MAX_SLAVES * 6 Bytes für MACs + 1 Byte Gültigkeitsflag + Puffer
#define EEPROM_SIZE (MAX_SLAVES * 6 + 1 + 1 + 10) // +1 für Rolle
#define EEPROM_MAC_ADDR 0 // Startadresse für die MACs im EEPROM
#define EEPROM_VALID_FLAG_ADDR (MAX_SLAVES * 6) // Adresse für Gültigkeitsflag (NACH allen MACs)
#define EEPROM_ROLE_ADDR (EEPROM_VALID_FLAG_ADDR + 1) // Adresse für die Rolle (NACH Gültigkeitsflag)

// U8g2 Objekt für ein 128x64 OLED mit SSD1306 Controller
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

// Definition der Zeitdauern (nach Benutzerbeschreibung und SpO, in Millisekunden)
// Zeiten für 3 Pfeile (AB/CD oder Standard-Einzel)
const unsigned long greenDuration3Arrows = 120000; // 2 Minuten = 120000 ms
// Zeiten für 6 Pfeile (Standard-Mannschaft oder -Einzel falls so geschossen wird)
const unsigned long greenDuration6Arrows = 240000; // 4 Minuten = 240000 ms

const unsigned long yellowDurationRest = 30000;     // 30 Sekunden Restzeit GELB (nach GRUEN)
const unsigned long yellowDurationPrep = 10000;     // 10 Sekunden Vorbereitung GELB (vor GRUEN)

// Die rote Phase ist warte-basiert, keine feste Dauer.

// Variablen für die Zeitmessung
unsigned long startTime; // Zeit des letzten Zustandswechsels

// Zustand der Ampel
State currentState = ROT;
// State previousState = ROT; // Nicht mehr direkt für Tonentscheidung in Slave benötigt

// NEU: Variablen für Keep-Alive und Verbindungsstatus
const unsigned long KEEP_ALIVE_INTERVAL = 5000; // Intervall für PING (ms)
const unsigned long CONNECTION_TIMEOUT = KEEP_ALIVE_INTERVAL + 2000; // Timeout (etwas länger als Intervall)
unsigned long lastPingSentTime = 0;      // Master: Wann wurde letzter PING gesendet?
unsigned long lastPongReceivedTime[MAX_SLAVES]; // Master: Wann kam letztes PONG an?
unsigned long lastMessageReceivedTime = 0; // Slave: Wann kam letzte Nachricht (PING oder State) an?
bool connectionOk = false; // Status der Verbindung (startet als nicht OK)

// Menü-Zustand
bool inMenu = false;
// NEU: "Zurueck" als letzter Menüpunkt
const char* menuItems[] = {"Schiesszeit", "Zielmodus", "Rolle", "Web Log", PSTR("Zurueck")};
// NEU: "Zurueck" als letzte Option in den Untermenüs
const char* shootingTimeOptions[] = {"3 Pfeile", "6 Pfeile", PSTR("Zurueck")};
const char* targetModes[] = {"Standard", "AB/CD", PSTR("Zurueck")};
const char* roleModes[] = {"Master", "Slave", PSTR("Zurueck")};
int currentMenuItem = 0;
int currentShootingTimeIndex = 0; // Index für Schießzeit 3/6 Pfeile
bool inSelectionMode = false; // NEU: Flag für Auswahlmodus
int selectionMenuItem = -1;   // NEU: Welcher Hauptmenüpunkt wird gerade ausgewählt?
int currentSelectionIndex = 0; // NEU: Index der Auswahl im Untermenü
int currentTargetModeIndex = 0; // Index für Zielmodus Standard/AB/CD
int currentRoleIndex = 0;


// Aktuelle Einstellungen
const char* shootingTimeSetting = shootingTimeOptions[0]; // Standard 3 Pfeile
const char* targetMode = targetModes[0];             // Standard Modus
const char* currentRole = roleModes[0];              // Master Rolle
bool isMaster = (currentRole == roleModes[0]); // "Master" ist roleModes[0]

// Aktuell gültige Grün-Dauer basierend auf shootingTimeSetting
unsigned long currentGreenDuration = greenDuration3Arrows;

// Zähler für den AB/CD Modus
int abCdPhase = 0; // 0 = AB Gruppe, 1 = CD Gruppe (aktuell schießend/wartend in DIESEM End)

// Neu: Variable für die Startgruppe der NÄCHSTEN Passe/End (AB/CD Modus)
bool isStartingGroupAB = true; // True, wenn AB das nächste End startet, False, wenn CD startet. Startet mit AB.

// Flags für manuelle Auslöser (gesetzt durch Joystick/Web, verarbeitet im Zustandsautomaten)
volatile bool manualStartTriggered = false; // Auslöser, um von ROT zu GELB_PREP zu wechseln
volatile bool manualStopTriggered = false;  // Auslöser, um von GELB_PREP/GRUEN/GELB_REST sofort zu ROT zu wechseln

// WLAN Konfiguration
const char* otaHostname = "BogenAmpel"; // Optional: Hostname für OTA

// OTA Timeout Konfiguration
unsigned long otaStartTime = 0;
const unsigned long otaTimeout = 30000; // 30 Sekunden Timeout für OTA (in Millisekunden)
bool otaTimedOut = false;
bool isOtaRunning = false; // Flag für den OTA-Zustand

// Webserver Objekt erstellen (Port 80 ist Standard für HTTP)
WebServer server(80);

// NEU: Globales Objekt für Serial Logging (Web und Hardware)
SerialAndWebLogger WebSerial(Serial);

// NEU: Encoder- und Button2-Objekte erstellen
ESP32Encoder encoder; // Das Encoder-Objekt
Button2 buttonEncoderSwitch(ENCODER_SW_PIN); // Taster des Encoders
Button2 buttonStartStop(startStopButtonPin);
Button2 buttonEmergency(emergencyButtonPin);

void handleEncoderSwitchPressed(Button2& btn); // Umbenannt für Klarheit
void handleStartStopPressed(Button2& btn);
void handleEmergencyPressed(Button2& btn);

// Forward-Deklarationen der Funktionen
void beep(unsigned int duration); // Grundfunktion, Tonfolgen in setState
void updateTimings();
void printCurrentSettings();
void drawMenu();
void updateLEDs(); // Aktualisiert die physischen LEDs
void setState(State newState, ToneType toneTypeToPlay = TONE_NONE); // <-- Parameter umbenannt
void processTrafficLightState(); // Enthält die Hauptlogik des Zustandsautomaten (nur Master)
void updateDisplay(); // Aktualisiert Display
void handleRoot(); // Webserver-Handler für die Root-Seite
void handleWebStart(); // Webserver-Handler zum Starten
void handleWebStop();  // Webserver-Handler zum Stoppen
void handleSetSchiesszeit(); // Webserver-Handler zum Setzen der Schießzeit
void handleSetZielmodus(); // Webserver-Handler zum Setzen des Zielmodus
void handleSetRolle(); // Webserver-Handler zum Setzen der Rolle
void handleNotFound(); // Webserver-Handler für unbekannte URLs
void handleWebReset(); // NEU: Handler zum Zuruecksetzen der Ampel/Serie
void triggerEmergencyStop(); // NEU: Funktion für Notstopp-Aktionen
void handleEmergencyStop(); // NEU: Web-Handler für Notstopp
void handleOtaUpdatePage(); // NEU: Handler für die GET /update Seite
void handleFirmwareUpload();      // NEU: Handler für POST /update (Daten)
void handleFirmwareUploadSuccess(); // NEU: Handler für POST /update (Erfolg)
void handleSetPeerMac(); // NEU: Handler zum Setzen der Peer MAC Adresse
void updateEspNowPeers(); // NEU: Helper zum Aktualisieren der ESP-NOW Peers
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status); // ESP-NOW Send Callback
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len); // ESP-NOW Receive Callback (Angepasste Signatur)
void handleStatusJson(); // NEU: Handler für Status-JSON
void loadRoleFromEEPROM(); // NEU: Lädt die Rolle aus dem EEPROM
void saveRoleToEEPROM();   // NEU: Speichert die Rolle im EEPROM
void handleGetSerialLog();      // NEU: Handler zum Abrufen des seriellen Logs
void handleToggleWebSerialLog(); // NEU: Handler zum Umschalten des Web-Serial-Logs
void handleClearWebSerialLog();  // NEU: Handler zum Löschen des Web-Serial-Logs


// Funktion für den Piezo-Summer (kann einfach tone() wrappen oder ungenutzt bleiben, da Töne in setState)
// Die eigentlichen Tonfolgen werden in setState() gespielt.
void beep(unsigned int duration) {
  // Optional: Kann für Notsignal etc. verwendet werden
  // tone(piezoPin, 1000, duration);
}

// --- EEPROM Funktionen ---
// Lädt ALLE Slave-MACs aus dem EEPROM in das globale slaveMacs Array
bool loadPeerMacsFromEEPROM() {
  if (EEPROM.read(EEPROM_VALID_FLAG_ADDR) == 0xAB) { // Prüfe Gültigkeitsflag
    WebSerial.println(F("Lade Slave MACs aus EEPROM:"));
    for (int i = 0; i < MAX_SLAVES; ++i) {
      WebSerial.printf("  Slave %d: ", i + 1);
      for (int j = 0; j < 6; ++j) {
        slaveMacs[i][j] = EEPROM.read(EEPROM_MAC_ADDR + i * 6 + j);
        WebSerial.printf("%02X", slaveMacs[i][j]);
        if (j < 5) WebSerial.print(":");
      }
      WebSerial.println();
    }
    return true;
  } else {
    WebSerial.println(F("Keine gültigen Slave MACs im EEPROM gefunden. Initialisiere mit FF:FF:FF:FF:FF:FF."));
    // Initialisiere das Array mit Broadcast/Leer, falls nichts gültiges da ist
    for (int i = 0; i < MAX_SLAVES; ++i) {
      memset(slaveMacs[i], 0xFF, 6);
    }
    return false;
  }
}

// Speichert das globale slaveMacs Array ins EEPROM
void savePeerMacsToEEPROM() {
  WebSerial.println(F("Speichere Slave MACs im EEPROM:"));
  for (int i = 0; i < MAX_SLAVES; ++i) {
    for (int j = 0; j < 6; ++j) {
      EEPROM.write(EEPROM_MAC_ADDR + i * 6 + j, slaveMacs[i][j]);
    }
  }
  EEPROM.write(EEPROM_VALID_FLAG_ADDR, 0xAB); // Setze Gültigkeitsflag
  EEPROM.commit(); // Änderungen speichern
  WebSerial.println(F(" - Gespeichert."));
}

// NEU: Lädt die Rolle (Master/Slave) aus dem EEPROM
void loadRoleFromEEPROM() {
  uint8_t roleIndex = EEPROM.read(EEPROM_ROLE_ADDR);
  if (roleIndex == 0 || roleIndex == 1) { // Gültiger Index (0=Master, 1=Slave)
    currentRoleIndex = roleIndex;
    currentRole = roleModes[currentRoleIndex];
    isMaster = (currentRoleIndex == 0);
    WebSerial.print(F("Rolle aus EEPROM geladen: "));
    WebSerial.println(currentRole);
  } else {
    WebSerial.println(F("Keine gültige Rolle im EEPROM gefunden oder erster Start."));
    WebSerial.println(F("Setze Standardrolle auf Master und speichere."));
    currentRoleIndex = 0; // Standard: Master
    currentRole = roleModes[currentRoleIndex];
    isMaster = true;
    saveRoleToEEPROM(); // Speichere den Standardwert
  }
}

// NEU: Speichert die aktuelle Rolle im EEPROM
void saveRoleToEEPROM() {
  EEPROM.write(EEPROM_ROLE_ADDR, currentRoleIndex);
  EEPROM.commit();
  WebSerial.print(F("Rolle im EEPROM gespeichert: "));
  WebSerial.println(currentRole);
}
// --- Ende EEPROM Funktionen ---


// Funktion zum Aktualisieren der Zeitdauern basierend auf Einstellung (3/6 Pfeile)
void updateTimings() {
  if (shootingTimeSetting == shootingTimeOptions[0]) { // "3 Pfeile"
    currentGreenDuration = greenDuration3Arrows;
  } else { // "6 Pfeile"
    currentGreenDuration = greenDuration6Arrows;
  }
  WebSerial.println(F("Zeiten aktualisiert:"));
  WebSerial.print(F("Grün-Dauer: "));
  WebSerial.println(currentGreenDuration);
  WebSerial.print(F("Gelb-Vorbereitung: "));
  WebSerial.println(yellowDurationPrep);
  WebSerial.print(F("Gelb-Restzeit: "));
  WebSerial.println(yellowDurationRest);
}

void printCurrentSettings() {
  WebSerial.println(F("Aktuelle Einstellungen:"));
  WebSerial.print(F("Schießzeit (Pfeile): "));
  WebSerial.println(shootingTimeSetting);
  WebSerial.print(F("Zielmodus: "));
  WebSerial.println(targetMode);
  WebSerial.print(F("Rolle: "));
  WebSerial.println(currentRole);
  if(targetMode == targetModes[1]) { // "AB/CD"
    WebSerial.print(F("Startet n\344chstes End mit AB?: ")); WebSerial.println(isStartingGroupAB ? F("Ja") : F("Nein")); // ä = \344
  }
}

void setup() {
  // Setze die LED-Pins als Ausgänge
  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(abLedPin, OUTPUT);
  pinMode(cdLedPin, OUTPUT);

  // NEU: KY-040 Encoder initialisieren
  // ESP32Encoder::useInternalWeakPullResistors = NONE; // Interne Pull-ups für CLK/DT aktivieren
  encoder.attachHalfQuad(ENCODER_CLK_PIN, ENCODER_DT_PIN);
  encoder.clearCount(); // Zähler auf 0 setzen
  WebSerial.println(F("KY-040 Encoder initialisiert."));

  // Setze den Piezo-Pin als Ausgang
  pinMode(piezoPin, OUTPUT);
  digitalWrite(piezoPin, LOW); // Stelle sicher, dass der Summer stumm ist


  // Initialisiere das U8g2 Display
  Wire.begin(); // Starte I2C
  u8g2.begin(); // Starte U8g2
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf); // Kleinere Schriftart (Alternative)
  u8g2.drawStr(2, 15, "Bogenampel");
  u8g2.sendBuffer();
  delay(1000); // Kurze Anzeige beim Start

  // Initialisiere die Startzeit (wird in setState gesetzt)

  // Serielle Kommunikation starten (für Debugging)
  Serial.begin(115200); // Das echte Serial muss immer noch initialisiert werden
  WebSerial.println(F("Bogenschießampel gestartet!"));

  // Initialzeiten und Einstellungen laden/anzeigen
  updateTimings(); // Setzt currentGreenDuration basierend auf shootingTimeSetting
  abCdPhase = 0; // Initial immer Phase AB im ersten End

  // EEPROM initialisieren und Peer MAC laden
  EEPROM.begin(EEPROM_SIZE);
  // Lade ALLE Slave MACs aus dem EEPROM in das globale Array
  loadPeerMacsFromEEPROM();
  // Lade die Rolle aus dem EEPROM (setzt currentRole, currentRoleIndex, isMaster)
  loadRoleFromEEPROM();
  printCurrentSettings(); // Jetzt mit der geladenen Rolle ausgeben

// --- NEU: Dynamische WLAN-Verbindung mit WiFiManager ---
  WiFi.mode(WIFI_STA);
  WebSerial.print(F("Eigene MAC Adresse: "));
  WebSerial.println(WiFi.macAddress());

  // WiFiManager initialisieren
  WiFiManager wm;

  // Setzt das Timeout für den Konfigurations-Access-Point auf 180 Sekunden
  wm.setConnectTimeout(180); 

  // Versucht, sich mit gespeicherten Daten zu verbinden.
  // Wenn das fehlschlägt, wird ein Access Point mit dem Namen "Bogenampel-Setup" gestartet.
  if (!wm.autoConnect("Bogenampel-Setup")) {
    WebSerial.println(F("Fehler beim Verbinden und Timeout erreicht. Neustart in 5 Sekunden."));
    delay(5000);
    ESP.restart();
  }

  // Wenn wir hier ankommen, ist die Verbindung erfolgreich.
  WebSerial.println(F("WLAN verbunden!"));
  WebSerial.print(F("IP Adresse: "));
  WebSerial.println(WiFi.localIP());

  // ESP-NOW initialisieren
  // Versuch, ESP-NOW zuerst zu deinitialisieren, falls es in einem inkonsistenten Zustand ist.
  // esp_now_deinit() gibt ESP_ERR_ESPNOW_NOT_INIT Zurueck, wenn ESP-NOW nicht initialisiert war,
  // was in diesem Fall in Ordnung ist und ignoriert werden kann.
  esp_now_deinit(); 
  if (esp_now_init() != ESP_OK) {
    WebSerial.println(F("Fehler beim Initialisieren von ESP-NOW"));
    return;
  }

// -- NEU: Broadcast-Peer für den MASTER hinzufügen, um Pairing-Anfragen zu empfangen --
if (isMaster) {
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfoBroadcast = {}; // Initialisieren
  memcpy(peerInfoBroadcast.peer_addr, broadcastAddress, 6);
  peerInfoBroadcast.channel = 0;
  peerInfoBroadcast.encrypt = false;

  if (esp_now_add_peer(&peerInfoBroadcast) != ESP_OK) {
    WebSerial.println(F("Fehler beim Hinzufügen des Broadcast-Peers für den Empfang von Anfragen."));
  } else {
    WebSerial.println(F("Master lauscht nun auf Broadcast-Adresse für Kopplungsanfragen."));
  }
}

  // Callback für gesendete Daten registrieren
  esp_now_register_send_cb(OnDataSent);
  // Registriere die geladenen Peers bei ESP-NOW
  updateEspNowPeers(); // Diese Funktion registriert die Peers aus slaveMacs
  WebSerial.printf("ESP-NOW initialisiert. %d Slaves registriert.\n", numRegisteredSlaves);

  // ArduinoOTA initialisieren
  ArduinoOTA.setHostname(otaHostname);
  // ArduinoOTA.setPassword("optional_password"); // Optionales Passwort für OTA

  ArduinoOTA.onStart([]() {
    WebSerial.println(F("Starte OTA Update"));
    isOtaRunning = true; // Setze das OTA-Running-Flag
    // Optional: Display aktualisieren oder LEDs ausschalten
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tf); // Kleinere Schriftart (Alternative)
    u8g2.drawStr(2, 15, "OTA Update...");
    u8g2.sendBuffer();
     // Alle LEDs ausschalten während OTA
     digitalWrite(redLedPin, LOW);
     digitalWrite(yellowLedPin, LOW);
     digitalWrite(greenLedPin, LOW);
     digitalWrite(abLedPin, LOW);
     digitalWrite(cdLedPin, LOW);
     noTone(piezoPin); // Summer ausschalten
  });
  ArduinoOTA.onEnd([]() {
    WebSerial.println(F("\nOTA Update beendet"));
    isOtaRunning = false; // Setze das OTA-Running-Flag Zurueck
    // Nach dem Update wird das Gerät normalerweise neu gestartet
    // ESP.restart(); // Wird normalerweise vom OTA Prozess gemacht
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // WebSerial.printf("Fortschritt: %u%%\r", (progress / (total / 100)));
    // Optional: Fortschritt auf Display anzeigen
     u8g2.clearBuffer();
     u8g2.setFont(u8g2_font_5x8_tf); // Kleinere Schriftart (Alternative)
     u8g2.setCursor(2, 15); // Start um 2 Pixel verschoben
     u8g2.printf("OTA: %u%%", (progress / (total / 100))); // u8g2.printf bleibt, da es nicht Serial ist
     u8g2.sendBuffer();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    WebSerial.printf("OTA Fehler [%u]: ", error);
    if (error == OTA_AUTH_ERROR) WebSerial.println(F("Authentifizierungsfehler"));
    else if (error == OTA_BEGIN_ERROR) WebSerial.println(F("Fehler beim Start"));
    else if (error == OTA_CONNECT_ERROR) WebSerial.println(F("Verbindungsfehler"));
    else if (error == OTA_RECEIVE_ERROR) WebSerial.println(F("Empfangsfehler"));
    else if (error == OTA_END_ERROR) WebSerial.println(F("Fehler beim Beenden"));
    otaTimedOut = true; // Setze das Timeout-Flag bei einem Fehler
    isOtaRunning = false; // Setze das OTA-Running-Flag Zurueck
    // Optional: Fehler auf Display anzeigen
     u8g2.clearBuffer();
     u8g2.setFont(u8g2_font_5x8_tf); // Kleinere Schriftart (Alternative)
     u8g2.drawStr(2, 15, "OTA Error!");
     u8g2.sendBuffer();
      // Optional: Rote LED blinken lassen bei Fehler?
     digitalWrite(redLedPin, HIGH);
     digitalWrite(yellowLedPin, LOW);
     digitalWrite(greenLedPin, LOW);
     digitalWrite(abLedPin, LOW);
     digitalWrite(cdLedPin, LOW);
  });
  ArduinoOTA.begin(); // Startet den OTA Service (auch HTTP Uploader)
  WebSerial.println(F("OTA bereit"));
  otaStartTime = millis(); // Starte den Timeout-Zähler, sobald OTA bereit ist

// -- NEUE LOGIK für SLAVE --
  if (!isMaster) {
    WebSerial.println(F("Slave-Modus: Sende Kopplungsanfrage per Broadcast..."));

    // Broadcast-Adresse als Peer hinzufügen, um eine Antwort vom Master zu erhalten
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      WebSerial.println(F("Fehler beim Hinzufügen des Broadcast-Peers"));
      return;
    }

    // Eigene MAC-Adresse in die Nachricht packen
    PairingMessage pairMsg;
    pairMsg.type = PAIRING_REQUEST;
    WiFi.macAddress(pairMsg.macAddr);

    // Nachricht senden
    esp_now_send(broadcastAddress, (uint8_t *) &pairMsg, sizeof(pairMsg));
  }

  // Webserver Routinen definieren
  server.on("/", handleRoot);         // Root-Seite
  server.on("/start", handleWebStart); // Start der Passe
  server.on("/stop", handleWebStop);   // Sofortiges Stoppen

  // Handler für Einstellungen
  server.on("/setSchiesszeit", handleSetSchiesszeit);
  server.on("/setZielmodus", handleSetZielmodus);
  server.on("/setRolle", handleSetRolle);
  server.on("/setPeerMac", handleSetPeerMac); // NEU: Handler für Peer MAC
  server.on("/status", handleStatusJson); // NEU: Handler für Status JSON
  server.on("/reset", handleWebReset); // NEU: Reset Handler
  server.on("/emergencyStop", handleEmergencyStop); // NEU: Notstopp Handler

  // Der *Upload*-Mechanismus für /update wird von ArduinoOTA bereitgestellt.
  server.on("/update", HTTP_GET, handleOtaUpdatePage); // Zeigt die Upload-Seite an
  server.on("/update", HTTP_POST, handleFirmwareUploadSuccess, handleFirmwareUpload); // Nimmt die hochgeladene Datei entgegen

  // NEU: Web-Serial-Log Endpunkte
  server.on("/getSerialLog", HTTP_GET, handleGetSerialLog);
  server.on("/toggleWebSerialLog", HTTP_GET, handleToggleWebSerialLog);
  server.on("/clearWebSerialLog", HTTP_GET, handleClearWebSerialLog);

  server.onNotFound(handleNotFound); // Für alle unbekannten URLs
  server.begin(); // Starte den Webserver
  WebSerial.println(F("Webserver gestartet"));

  // Setze den initialen Zustand auf ROT und sende ihn (Master) oder warte (Slave)
  // abCdPhase und isStartingGroupAB sind schon initialisiert
  if (isMaster) { // Nur der Master setzt den initialen Zustand aktiv
     setState(ROT, TONE_NONE); // Setzt den initialen Zustand (spielt keinen Ton bei Systemstart), sendet ESP-NOW
  }

  // Callback für empfangene Daten registrieren (NACH dem ersten setState, um sicherzustellen, dass der Slave bereit ist)
  esp_now_register_recv_cb(OnDataRecv);

// Button2 Handler initialisieren
  buttonEncoderSwitch.setPressedHandler(handleEncoderSwitchPressed); // Umbenannter Handler
  buttonStartStop.setPressedHandler(handleStartStopPressed);
  buttonEmergency.setPressedHandler(handleEmergencyPressed);
}

void loop() {
  // Immer OTA und Webserver bearbeiten
  ArduinoOTA.handle();
  server.handleClient();

  // NEU: Logik zum Auslesen des KY-040 Drehgebers
  if (inMenu) { // Den Encoder nur auswerten, wenn wir im Menü sind
    long newEncoderValue = encoder.getCount();

    if (newEncoderValue != oldEncoderValue) {
      if (newEncoderValue > oldEncoderValue) {
        // Encoder wurde im Uhrzeigersinn gedreht (entspricht "Runter")
        WebSerial.println(F("Encoder: Runter gedreht."));
        if (!inSelectionMode) { // Im Hauptmenü navigieren
            currentMenuItem = (currentMenuItem < (sizeof(menuItems) / sizeof(menuItems[0]) - 1)) ? currentMenuItem + 1 : 0; 
        } else { // Im Auswahlmodus navigieren
            int numOptions = 0;
            if (selectionMenuItem == 0) numOptions = sizeof(shootingTimeOptions) / sizeof(shootingTimeOptions[0]); 
            else if (selectionMenuItem == 1) numOptions = sizeof(targetModes) / sizeof(targetModes[0]); 
            else if (selectionMenuItem == 2) numOptions = sizeof(roleModes) / sizeof(roleModes[0]); 
            currentSelectionIndex = (currentSelectionIndex < numOptions - 1) ? currentSelectionIndex + 1 : 0; 
        }
        drawMenu(); // Menü neu zeichnen
      } else {
        // Encoder wurde gegen den Uhrzeigersinn gedreht (entspricht "Hoch")
        WebSerial.println(F("Encoder: Hoch gedreht."));
        if (!inSelectionMode) { // Im Hauptmenü navigieren
            currentMenuItem = (currentMenuItem > 0) ? currentMenuItem - 1 : (sizeof(menuItems) / sizeof(menuItems[0]) - 1); 
        } else { // Im Auswahlmodus navigieren
            int numOptions = 0;
            if (selectionMenuItem == 0) numOptions = sizeof(shootingTimeOptions) / sizeof(shootingTimeOptions[0]); 
            else if (selectionMenuItem == 1) numOptions = sizeof(targetModes) / sizeof(targetModes[0]); 
            else if (selectionMenuItem == 2) numOptions = sizeof(roleModes) / sizeof(roleModes[0]); 
            currentSelectionIndex = (currentSelectionIndex > 0) ? currentSelectionIndex - 1 : numOptions - 1; 
        }
        drawMenu(); // Menü neu zeichnen
      }
      oldEncoderValue = newEncoderValue; // Neuen Wert als alten speichern
      encoder.clearCount(); // Zähler Zuruecksetzen, um Überlauf zu vermeiden und relative Bewegung zu nutzen
      oldEncoderValue = 0;
    }
  }
  // OTA Timeout Überprüfung
  if (isOtaRunning && (millis() - otaStartTime > otaTimeout) && !otaTimedOut) {
     WebSerial.println(F("OTA Timeout erreicht."));
     otaTimedOut = true; // Markiere als getimeoutet
     isOtaRunning = false; // Setze Flag Zurueck (oder lass OTAErrorHandler das machen)
     // Der OTA onError Handler macht bereits Display-Update und setzt isOtaRunning/otaTimedOut.
  }


  // Hauptprogrammlogik nur ausführen, wenn KEIN OTA-Update läuft
  if (!isOtaRunning) {

     // --- Keep-Alive Logik ---
     unsigned long now = millis();
     if (isMaster) {
        // Master: PING senden, wenn Intervall abgelaufen
        if (now - lastPingSentTime >= KEEP_ALIVE_INTERVAL) {
           lastPingSentTime = now;
           // Sende PING an ALLE registrierten Slaves
           WebSerial.printf("KeepAlive: Sende PING an %d Slaves...\n", numRegisteredSlaves);
           KeepAliveMsg pingMsg = {PING};
           for (int i = 0; i < numRegisteredSlaves; ++i) { // Sende nur an die tatsächlich registrierten
              esp_now_send(slaveMacs[i], (uint8_t *) &pingMsg, sizeof(pingMsg));
           }
           // Nach dem Senden eines Pings den Pong-Timer nicht sofort Zuruecksetzen,
           // sondern auf die Antwort warten.
        }
        // Master: Verbindung prüfen (Timeout seit letztem PONG)
        if (isMaster) { // Only master checks connection status of slaves
    unsigned long now = millis();
    bool anySlaveConnected = false;
    for (int i = 0; i < numRegisteredSlaves; ++i) {
        if (now - lastPongReceivedTime[i] < CONNECTION_TIMEOUT) {
            anySlaveConnected = true;
            break; // Found at least one connected slave, no need to check further
        }
    }
    connectionOk = anySlaveConnected; // Update the global connection status
}
     } else {
        // Slave: Verbindung prüfen (Timeout seit letzter Nachricht vom Master)
        connectionOk = (now - lastMessageReceivedTime < CONNECTION_TIMEOUT);
     }
     
     // --- Zustandsautomaten-Logik (nur Master) ---
     // Prozessiere die Ampellogik basierend auf Zeit und manuellen Triggern
     // Diese Funktion ruft bei Bedarf setState() auf
     if (isMaster) {
        processTrafficLightState();
     }
     // --- Ende Zustandsautomaten-Logik ---


     // --- Ausgabe Aktualisierung (Display, LEDs) ---
     // Aktualisiere Display und LEDs basierend auf dem aktuellen Zustand und Modus
     // Dies geschieht sowohl im Master als auch im Slave (nach Empfang oder Logik-Update)
     // Das Display wird nur aktualisiert, wenn nicht im Menü und nicht OTA busy/error
     if (!inMenu && !isOtaRunning && !otaTimedOut) {
       updateDisplay();
     }
     // Die LEDs MÜSSEN immer den aktuellen Zustand anzeigen, außer bei OTA Lauf
     if (!isOtaRunning) {
        updateLEDs();
     }

  // --- Button2 Loop Aufrufe ---
  // Diese müssen regelmäßig aufgerufen werden, damit die Button2 Bibliothek die Tasterzustände prüfen kann
  buttonEncoderSwitch.loop();
  buttonStartStop.loop();
  buttonEmergency.loop();


  } // Ende if (!isOtaRunning)
} // Ende loop()

// Zentrale Funktion für Notstopp-Aktionen
void triggerEmergencyStop() {
    WebSerial.println(F("!!! TRIGGERING EMERGENCY STOP !!!"));
    // Sofortige Aktionen (Master & Slave)
    currentState = ROT; // Sofort auf ROT
    abCdPhase = 0;      // Sequenz Zuruecksetzen
    isStartingGroupAB = true;
    startTime = millis(); // Timer neu starten für ROT
    // Sofort Notsignal spielen (blockierend)
    noTone(piezoPin);
    tone(piezoPin, 1500, 100); delay(150); noTone(piezoPin); delay(50);
    tone(piezoPin, 1500, 100); delay(150); noTone(piezoPin); delay(50);
    tone(piezoPin, 1500, 100); delay(150); noTone(piezoPin); delay(50);
    tone(piezoPin, 1500, 100); noTone(piezoPin);
    // Wenn Master, Zustand senden (Ton wird in setState nicht erneut gespielt)
    if (isMaster) { setState(ROT, TONE_EMERGENCY); }
    // Display/LEDs werden in der nächsten loop()-Iteration aktualisiert
}

// Enthält die Hauptlogik des Zustandsautomaten (nur Master!)
void processTrafficLightState() {
  if (!isMaster) return;

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime; // Zeit seit dem letzten Zustandswechsels

  // --- Manuelle Stop-Logik (konsolidiert) ---
  // Prüfen, ob ein manueller Stopp ausgelöst wurde, während die Ampel aktiv ist
  if (manualStopTriggered && (currentState == GELB_PREP || currentState == GRUEN || currentState == GELB_REST)) {
      manualStopTriggered = false; // Trigger verbrauchen
      WebSerial.println(F("Master: Manual Stop Trigger erhalten."));

      ToneType toneToPlay = TONE_NONE;
      bool isEndOfEnd = false;
      int finishedPhase = abCdPhase; // Die Phase, die gerade unterbrochen wurde

      // Bestimme den korrekten End-Ton und ob das End vorbei ist
      if (targetMode == targetModes[1]) { // "AB/CD"
          if (finishedPhase == 0) { // AB wurde unterbrochen
              if (!isStartingGroupAB) { isEndOfEnd = true; } // Wenn CD gestartet hat, ist das End jetzt vorbei
          } else { // CD wurde unterbrochen
              if (isStartingGroupAB) { isEndOfEnd = true; } // Wenn AB gestartet hat, ist das End jetzt vorbei
          }
          // Ton spielen basierend auf isEndOfEnd
          if (isEndOfEnd) {
              toneToPlay = TONE_END_OF_END; // Dreimaliges Signal für das Ende eines AB/CD Endes
          } else {
              toneToPlay = TONE_GROUP_CHANGE; // Zweimaliges Signal für Gruppenwechsel
          }

      } else { // Standard Modus unterbrochen
          toneToPlay = TONE_END_OF_END; // Dreifachton
          isEndOfEnd = true; // Im Standard Modus ist es immer das Ende des Ends
      }

      // Spielen des ermittelten Tons (blockierend)
      noTone(piezoPin);
      switch(toneToPlay) {
            case TONE_END_OF_END:           // Dreimaliges Pfeifen: Schießzeit beendet, Trefferaufnahme
            tone(piezoPin, 1000, 50); delay(150); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin);
                break;
            case TONE_GROUP_CHANGE: // Zweimaliges Pfeifen
               tone(piezoPin, 1000, 50); delay(150); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin);
                break;
            case TONE_NONE:
               // Kein Ton
                break;
           default: break; // Sollte mit dieser Logik nicht vorkommen
      }

      // Aktualisiere Phase/Startgruppe für den *nächsten* Durchgang
      if (targetMode == targetModes[1]) { // "AB/CD"
           if (isEndOfEnd) {
              // End ist vorbei. Nächste abCdPhase ist die Startgruppe des NÄCHSTEN Ends.
              isStartingGroupAB = !isStartingGroupAB; // Toggle die Startgruppe
              abCdPhase = isStartingGroupAB ? 0 : 1;  // Setze Phase entsprechend
              WebSerial.print(F("Master (Manual Stop): End beendet. Nächste Passe startet mit ")); WebSerial.println(isStartingGroupAB ? F("AB") : F("CD"));
           } else {
              // End ist NICHT vorbei. Nächste abCdPhase ist die andere Gruppe in diesem End.
              abCdPhase = (finishedPhase == 0) ? 1 : 0; // Wechsle zur anderen Gruppe
              WebSerial.print(F("Master (Manual Stop): Gruppe fertig, End läuft weiter. Nächste Gruppe: ")); WebSerial.println(abCdPhase == 0 ? F("AB") : F("CD"));
           }
      } else { // Standard Modus Reset nach manuellem Stop
           abCdPhase = 0;
           isStartingGroupAB = true;
           WebSerial.println(F("Master (Manual Stop): Standard Modus beendet."));
      }

      // Gehe zu ROT, Ton wurde bereits gespielt
      setState(ROT, TONE_NONE); // Ton wurde schon oben gespielt, hier TONE_NONE senden

      // Wichtig: Verlasse die Funktion hier, damit die switch-Anweisung nicht mehr ausgeführt wird
      return;
  }
  // --- Ende Manuelle Stop-Logik ---

  switch (currentState) {
    case ROT:
      // Im ROT-Zustand warten wir auf einen manuellen Start-Trigger.
      if (manualStartTriggered) {
        manualStartTriggered = false; // Trigger verbrauchen
        WebSerial.print(F("DEBUG (ROT->Start): Aktuelle abCdPhase=")); WebSerial.print(abCdPhase);
        WebSerial.print(F(", isStartingGroupAB=")); WebSerial.println(isStartingGroupAB);
        WebSerial.println(F("Master: Start Trigger erhalten in ROT."));
        setState(GELB_PREP, TONE_GO_TO_LINE); // Gehe zu GELB_PREP (10s Vorbereitung), spiele Zweifachton
      }
      // Im ROT-Zustand gibt es keine automatischen Zeitübergänge.
      break;

    case GELB_PREP:
      // Im GELB_PREP Zustand laufen die 10 Sekunden Vorbereitung.
      // Übergang zu GRUEN nach 10 Sekunden.
      if (elapsedTime >= yellowDurationPrep) {
         WebSerial.println(F("Master: Gelb_Prep Zeit abgelaufen. Wechsel zu GRUEN."));
        setState(GRUEN, TONE_SHOOT_START); // Gehe zu GRUEN (Haupt-Schießzeit), spiele Einmalton
      }
       // --> Die Logik wurde nach oben verschoben <--
      break;

    case GRUEN:
      // Im GRUEN-Zustand läuft die Schießzeit.
      // Wir überprüfen auf manuellen Stop ODER den automatischen Übergang zu GELB_REST.

      // --> Die Logik wurde nach oben verschoben <--
      
      // Prüfen auf automatischen Übergang zu GELB_REST (30 Sek vor Ende der GRUEN-Zeit)
      if (elapsedTime >= (currentGreenDuration - yellowDurationRest)) {
         WebSerial.println(F("Master: Gruenzeit fast abgelaufen. Wechsel zu GELB_REST."));
        setState(GELB_REST, TONE_NONE); // Gehe zu GELB_REST (30s Restzeit), kein Ton
      }
      break;

    case GELB_REST:
      // Im GELB_REST-Zustand sind die letzten 30 Sekunden.
      // Wir überprüfen auf manuellen Stop ODER den automatischen Übergang zu ROT.

      // --> Die Logik wurde nach oben verschoben <--

      // Prüfen auf automatischen Übergang zu ROT (nachdem die GELB_REST-Zeit abgelaufen ist)
      if (elapsedTime >= yellowDurationRest) { // GELB_REST ist 30 Sekunden lang
        WebSerial.println(F("Master: Gelb_Rest Zeit abgelaufen."));
        WebSerial.print(F("DEBUG (GELB_REST Ende): Vor Berechnung: abCdPhase=")); WebSerial.print(abCdPhase);
        WebSerial.print(F(", isStartingGroupAB=")); WebSerial.println(isStartingGroupAB);

        ToneType toneToPlay = TONE_NONE; // Initialisiere mit TONE_NONE
        int finishedPhase = abCdPhase; // Phase, die gerade schießt/fertig ist
        bool isEndOfEnd = false;      // Flag, ob das gesamte End vorbei ist

        // Bestimme den Ton basierend auf der Gruppe, die gerade fertig ist (finishedPhase)
        if (targetMode == targetModes[1]) { // "AB/CD"
           if (finishedPhase == 0) { // AB Gruppe ist fertig
               // AB ist fertig. War AB die Startgruppe (isStartingGroupAB)?
               // Wenn NEIN, dann war CD die Startgruppe, und AB war die zweite Gruppe. Das End ist vorbei.
               if (!isStartingGroupAB) {
                   isEndOfEnd = true; // End ist vorbei
               }
               // Wenn JA, dann war AB die Startgruppe, und AB ist die erste Gruppe, die fertig ist. End ist noch nicht vorbei.
           } else { // CD Gruppe ist fertig (finishedPhase == 1)
               // CD ist fertig. War CD die Startgruppe (isStartingGroupAB)?
               // Wenn JA, dann war CD die Startgruppe, und CD ist die erste Gruppe, die fertig ist. End ist noch nicht vorbei.
                if (isStartingGroupAB) { // Wenn AB die Startgruppe war, dann ist CD die zweite Gruppe
                   isEndOfEnd = true; // End ist vorbei
               }
           }
           // Ton spielen basierend auf isEndOfEnd
           if (isEndOfEnd) {
               toneToPlay = TONE_END_OF_END; // Dreimaliges Signal für das Ende eines AB/CD Endes
           } else {
               toneToPlay = TONE_GROUP_CHANGE; // Zweimaliges Signal für Gruppenwechsel
           }

           // Setze die abCdPhase für den NÄCHSTEN DURCHGANG (entweder die andere Gruppe in diesem End, oder die Startgruppe des nächsten Ends)
           if (isEndOfEnd) {
           WebSerial.print(F("DEBUG: isEndOfEnd ist TRUE. Vor Toggle: isStartingGroupAB=")); WebSerial.println(isStartingGroupAB); // NEUE DEBUG ZEILE

              // End ist vorbei. Nächste abCdPhase ist die Startgruppe des NÄCHSTEN Ends.
              isStartingGroupAB = !isStartingGroupAB; // Toggle die Startgruppe für das nächste End
               if (isStartingGroupAB) {
                   abCdPhase = 0; // Nächstes End startet mit AB
               } else {
                   abCdPhase = 1; // Nächstes End startet mit CD
               }
            WebSerial.print(F("DEBUG: isEndOfEnd ist TRUE. Nach Toggle: isStartingGroupAB=")); WebSerial.println(isStartingGroupAB); // NEUE DEBUG ZEILE

               WebSerial.print(F("Master: End beendet. N\344chste Passe startet mit ")); WebSerial.println(isStartingGroupAB ? F("AB") : F("CD"));
               WebSerial.print(F("DEBUG (GELB_REST Ende): Nach EndOfEnd: Neue abCdPhase=")); WebSerial.print(abCdPhase);
               WebSerial.print(F(", Neue isStartingGroupAB=")); WebSerial.println(isStartingGroupAB);

           } else {
              // End ist NICHT vorbei. Nächste abCdPhase ist die andere Gruppe in diesem End.
              if (finishedPhase == 0) { // AB war fertig, nächste ist CD
                  abCdPhase = 1;
              } else { // CD war fertig, nächste ist AB
                  abCdPhase = 0;
              }
              WebSerial.print(F("Master: Gruppe fertig, End l\344uft weiter. N\344chste Gruppe: ")); WebSerial.println(abCdPhase == 0 ? F("AB") : F("CD"));
              WebSerial.print(F("DEBUG (GELB_REST Ende): Nach Gruppenwechsel: Neue abCdPhase=")); WebSerial.print(abCdPhase);
              WebSerial.print(F(", isStartingGroupAB=")); WebSerial.println(isStartingGroupAB); // Bleibt gleich
           }


        } else { // Standard Modus fertig
           toneToPlay = TONE_END_OF_END; // Dreifachton
           abCdPhase = 0; // Phase bleibt 0 im Standard Modus
           isEndOfEnd = true; // Ein End im Standard Modus ist immer ein ganzes End
           // isStartingGroupAB wird im Standard Modus nicht verwendet, aber wir setzen es zur Sicherheit auf true
           isStartingGroupAB = true;
           WebSerial.println(F("Master: Standard Modus fertig."));
           WebSerial.print(F("DEBUG (GELB_REST Ende): Nach Standard: Neue abCdPhase=")); WebSerial.print(abCdPhase);
           WebSerial.print(F(", Neue isStartingGroupAB=")); WebSerial.println(isStartingGroupAB);
        }

        // Spielen des Tons direkt hier (blockierend)
        noTone(piezoPin); // Alten Ton stoppen
         switch(toneToPlay) {
             case TONE_END_OF_END: // Dreimaliges Pfeifen
                 tone(piezoPin, 1000, 50); delay(150); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin);
                 break;
             case TONE_GROUP_CHANGE: // Zweimaliges Pfeifen
                 tone(piezoPin, 1000, 50); delay(150); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin);
                 break;
             case TONE_NONE:
                 // Kein Ton
                 break;
             default: // Sollte für diese Übergänge nicht TONE_NONE sein
                 break;
         }
         // Warten auf Tonende ist durch delay gegeben.


        // Zustand wechseln (NRF Sendung passiert in setState)
        setState(ROT, TONE_NONE); // Gehe zu ROT, sende TONE_NONE im NRF, da Ton schon gespielt
      }
      break; // break for GELB_REST case
  }
}

// Wechselt den Zustand der Ampel (nur im Master aufrufen oder im Slave durch Callback!)
// Setzt Timer neu, spielt spezifische Töne (NUR DIE NICHT-BLOCKIERENDEN), sendet NRF
void setState(State newState, ToneType toneTypeToPlay ) {
  // Zustandswchsel immer senden und Timer setzen
  if (newState != currentState || !isMaster) { // Zustand hat sich geändert ODER wir sind Slave (dann immer aktualisieren)
     State oldState = currentState; // Speichern des alten Zustands für Logik

     currentState = newState; // Zustand aktualisieren
     startTime = millis();    // Timer für den NEUEN Zustand starten

     if (isMaster) {
        WebSerial.print(F("Master: Zustand gewechselt von ")); WebSerial.print(oldState); WebSerial.print(F(" zu ")); WebSerial.print(newState);
        WebSerial.print(F(" (Tone Type: ")); WebSerial.print(toneTypeToPlay ); WebSerial.println(F(") - Sende ESP-NOW."));

        // Akustische Signale basierend auf dem übergebenen ToneType (NUR TÖNE, DIE HIER GESPIELT WERDEN)
        // Die Töne TONE_GROUP_CHANGE und TONE_END_OF_END werden jetzt in processTrafficLightState gespielt (blockierend).
        // Hier spielen wir nur die nicht-blockierenden (oder kürzeren) Töne.
        noTone(piezoPin); // Alten Ton stoppen

        switch(toneTypeToPlay ) { // Switch über den Parameter toneTypeToPlay
          case TONE_GO_TO_LINE: // Zweimaliges Pfeifen: Zur Schießlinie gehen
            WebSerial.println(F("Ton (setState): Zweimal (ROT -> GELB_PREP, Zur Schiesslinie)"));
            tone(piezoPin, 1000, 50); delay(150); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin);
            break;
          case TONE_SHOOT_START: // Einmaliges Pfeifen: Schießbeginn
            WebSerial.println(F("Ton (setState): Einmal (GELB_PREP -> GRUEN, Schiessbeginn)"));
            tone(piezoPin, 1000, 100); noTone(piezoPin);
            break;
          case TONE_END_OF_END: // Dreimaliges Pfeifen: Pfeile hohlen
            WebSerial.println(F("Ton (setState): Dreimal (Pfeile hohlen)"));
            tone(piezoPin, 1000, 50); delay(150); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin); tone(piezoPin, 1000, 50);
            break;
          case TONE_EMERGENCY: // Wird bereits in triggerEmergencyStop() gespielt, hier nur loggen/sende
             WebSerial.println(F("Ton (setState): Notfall (bereits gespielt)"));
          break;
             // Töne TONE_GROUP_CHANGE und TONE_END_OF_END werden in processTrafficLightState gespielt (blockierend)
             // TONE_NONE
             default: // TONE_NONE oder andere Typen, die hier nicht gespielt werden
                noTone(piezoPin); // Keine Töne
                break;
         }

         // Sende den neuen Zustand, die aktuelle Phase, den Ton-Typ UND die Startgruppen-Info per ESP-NOW
         currentStateData.state = currentState; // Sende den NEUEN Zustand
         currentStateData.abCdPhase = abCdPhase; // Sende die aktuelle AB/CD Phase (wurde in processTrafficLightState gesetzt)
         currentStateData.toneType  = toneTypeToPlay ;           // Sende den ToneType (auch wenn Ton in processTrafficLightState gespielt wurde)
         currentStateData.isStartingGroupAB = isStartingGroupAB; // Sende die Startgruppen-Info
         currentStateData.shootingTimeIndex = currentShootingTimeIndex; // Sende den Index der Schießzeit
         currentStateData.targetModeIndex = currentTargetModeIndex;     // Sende den Index des Zielmodus

         // Sende die Daten via ESP-NOW an ALLE registrierten Slaves
         WebSerial.printf("Master sendet ESP-NOW Zustand an %d Slaves: State=%d, Phase=%d, Ton=%d, StartAB=%d\n",
                       numRegisteredSlaves, currentState, abCdPhase, toneTypeToPlay , isStartingGroupAB);

         for (int i = 0; i < numRegisteredSlaves; ++i) { // Sende nur an die tatsächlich registrierten
            esp_err_t result = esp_now_send(slaveMacs[i], (uint8_t *) &currentStateData, sizeof(AmpelStateMessage)); // Wichtig: sizeof(AmpelStateMessage) verwenden!
            if (result != ESP_OK) {
               WebSerial.printf("  -> Fehler beim Senden an Slave %d (%02X:%02X:%02X:%02X:%02X:%02X)\n", i+1,
                             slaveMacs[i][0], slaveMacs[i][1], slaveMacs[i][2], slaveMacs[i][3], slaveMacs[i][4], slaveMacs[i][5]);
            }
         }
     } // Ende if(isMaster)
     // Die LEDs und das Display werden am Ende von loop() aktualisiert.
  } else if (isMaster) { // Zustand hat sich NICHT geändert, aber wir sind Master
      // Wenn ein Ton gespielt wurde OHNE Zustandswechsel (z.B. Notsignal), sende den Zustand trotzdem,
      // damit der Slave den Ton spielen kann (der Ton-Typ wird im currentStateData mitgesendet).
      if (toneTypeToPlay != TONE_NONE) {
         WebSerial.print(F("Master: Sende Zustand wegen Ton ")); WebSerial.print(toneTypeToPlay ); WebSerial.println(F(" OHNE Zustandswechsel."));

         // Sende den aktuellen Zustand mit dem neuen Ton-Typ
         currentStateData.state = currentState;
         currentStateData.abCdPhase = abCdPhase;
         currentStateData.toneType  = toneTypeToPlay ;
         currentStateData.isStartingGroupAB = isStartingGroupAB;
         currentStateData.shootingTimeIndex = currentShootingTimeIndex;
         currentStateData.targetModeIndex = currentTargetModeIndex;

         // Sende die Daten via ESP-NOW an ALLE registrierten Slaves
         WebSerial.printf("Master sendet ESP-NOW Zustand (Ton-Update) an %d Slaves: State=%d, Phase=%d, Ton=%d, StartAB=%d\n",
                       numRegisteredSlaves, currentState, abCdPhase, toneTypeToPlay , isStartingGroupAB);

         for (int i = 0; i < numRegisteredSlaves; ++i) {
            esp_err_t result = esp_now_send(slaveMacs[i], (uint8_t *) &currentStateData, sizeof(AmpelStateMessage)); // Wichtig: sizeof(AmpelStateMessage) verwenden!
            if (result != ESP_OK) {
               WebSerial.printf("  -> Fehler beim Senden an Slave %d (%02X:%02X:%02X:%02X:%02X:%02X)\n", i+1,
                             slaveMacs[i][0], slaveMacs[i][1], slaveMacs[i][2], slaveMacs[i][3], slaveMacs[i][4], slaveMacs[i][5]);
            }
         }
      }
  }
}


// Aktualisiert die physischen LED-Ausgänge basierend auf dem aktuellen Zustand und Modus
void updateLEDs() {
  // Zuerst alle Hauptampel LEDs aus
  digitalWrite(redLedPin, LOW);
  digitalWrite(yellowLedPin, LOW);
  digitalWrite(greenLedPin, LOW);

  // Hauptampel LEDs basierend auf dem Zustand
  switch (currentState) {
    case ROT:
      digitalWrite(redLedPin, HIGH);
      break;
    case GELB_PREP: // Beide GELB Zustände schalten die GELB LED
    case GELB_REST:
      digitalWrite(yellowLedPin, HIGH);
      break;
    case GRUEN:
      digitalWrite(greenLedPin, HIGH);
      break;
  }

  // AB/CD LEDs basierend auf Modus, Phase UND Haupt-Zustand
  // Sie leuchten nur WÄHREND der aktiven Schießphase (GELB_PREP, GRUEN, GELB_REST) der jeweiligen Gruppe
  digitalWrite(abLedPin, LOW); // Standardmäßig aus
  digitalWrite(cdLedPin, LOW); // Standardmäßig aus

  if (targetMode == targetModes[1]) { // "AB/CD"
    // LEDs leuchten nur während der GELB_PREP, GRUEN, GELB_REST Zustände
    if (currentState != ROT) {
      // Die abCdPhase hier im updateLEDs zeigt die Gruppe an, die gerade schießt oder deren Vorbereitung/Restzeit läuft.
      if (abCdPhase == 0) { // AB Phase ist dran
        digitalWrite(abLedPin, HIGH);
      } else { // CD Phase ist dran
        digitalWrite(cdLedPin, HIGH);
      }
    }
  }
  // Im Standard Modus bleiben AB/CD LEDs aus.
  // Im ROT Zustand bleiben AB/CD LEDs aus.
}


  void updateDisplay() {
    // Wenn das Menü aktiv ist, wird das Display durch drawMenu() aktualisiert.
    // Hier aktualisieren wir das Display nur im normalen Betriebsmodus.
    if (inMenu) {
      return; // Display wird von drawMenu() verwaltet
    }
  
    // Wenn OTA läuft oder Fehler hat, hat der OTA Handler das Display bereits übernommen.
    if (isOtaRunning || otaTimedOut) {
       return; // Display bleibt so, wie vom OTA Handler oder onError gesetzt.
    }
  
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tf);

    char buf[48]; // Buffer for formatting display strings
    const char* stateTextCStr = "";
    // Anzeige des aktuellen Zustands
    unsigned long currentDurationForTime = 0; // Dauer für die Zeitberechnung
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;
  
    switch (currentState) {
      case ROT:
        { // Scope für ROT beginnen
          stateTextCStr = "ROT";
          // Im ROT Zustand keine Zeit anzeigen, sondern Warte-Info oder nächste Phase / Startgruppe
          if (targetMode == targetModes[1]) { // "AB/CD"
             snprintf(buf, sizeof(buf), "N\344chste Grp: %s", (abCdPhase == 0 ? "AB" : "CD"));
             u8g2.drawStr(2, 25, buf);
             snprintf(buf, sizeof(buf), "Start n. End: %s", (isStartingGroupAB ? "AB" : "CD"));
             u8g2.drawStr(2, 35, buf);
          } else {
             u8g2.drawStr(2, 25, "Warten...");
             // u8g2.drawStr(2, 35, ""); // Zeile 35 leer lassen
          }
        } // Scope für ROT beenden
          break;
      case GELB_PREP:
        { // Scope für GELB_PREP beginnen
          stateTextCStr = "GELB (Vorb.)";
          currentDurationForTime = yellowDurationPrep;
           // Verbleibende Zeit anzeigen (Sekunden)
           int remainingSecondsPrep = 0;
          if (currentDurationForTime > elapsedTime) {
               remainingSecondsPrep = (currentDurationForTime - elapsedTime) / 1000;
           } else {
               remainingSecondsPrep = 0;
           }
          snprintf(buf, sizeof(buf), "Zeit: %ds", remainingSecondsPrep);
          u8g2.drawStr(2, 25, buf);
          // Zeige aktive Gruppe im AB/CD Modus
          if (targetMode == targetModes[1]) { // "AB/CD"
             snprintf(buf, sizeof(buf), "Gruppe: %s", (abCdPhase == 0 ? "AB" : "CD"));
             u8g2.drawStr(2, 35, buf);
          } else {
             // u8g2.drawStr(2, 35, ""); // Leer im Standard Modus
          }
        } // Scope für GELB_PREP beenden
          break;
      case GRUEN:
        { // Scope für GRUEN beginnen
          stateTextCStr = "GRUEN";
          currentDurationForTime = currentGreenDuration;
           // Verbleibende Zeit anzeigen (MM:SS)
           int remainingSecondsGruen = 0;
          if (currentDurationForTime > elapsedTime) {
               remainingSecondsGruen = (currentDurationForTime - elapsedTime) / 1000;
           } else {
               remainingSecondsGruen = 0;
           }
          int minutes = remainingSecondsGruen / 60;
          int seconds = remainingSecondsGruen % 60;
          snprintf(buf, sizeof(buf), "Zeit: %02d:%02d", minutes, seconds);
          u8g2.drawStr(2, 25, buf);
          // Zeige aktive Gruppe im AB/CD Modus
          if (targetMode == targetModes[1]) { // "AB/CD"
             snprintf(buf, sizeof(buf), "Gruppe: %s", (abCdPhase == 0 ? "AB" : "CD"));
             u8g2.drawStr(2, 35, buf);
          } else {
             // u8g2.drawStr(2, 35, ""); // Leer im Standard Modus
          }
        } // Scope für GRUEN beenden
          break;
      case GELB_REST:
        { // Scope für GELB_REST beginnen
          stateTextCStr = "GELB (Rest)";
          currentDurationForTime = yellowDurationRest;
          // Verbleibende Zeit anzeigen (Sekunden)
           int remainingSecondsRest = 0;
           if (currentDurationForTime > elapsedTime) {
               remainingSecondsRest = (currentDurationForTime - elapsedTime) / 1000;
           } else {
               remainingSecondsRest = 0;
           }
          snprintf(buf, sizeof(buf), "Zeit: %3ds", remainingSecondsRest);
          u8g2.drawStr(2, 25, buf);
          // Zeige aktive Gruppe im AB/CD Modus
          if (targetMode == targetModes[1]) { // "AB/CD"
             snprintf(buf, sizeof(buf), "Gruppe: %s", (abCdPhase == 0 ? "AB" : "CD"));
             u8g2.drawStr(2, 35, buf);
          } else {
             // u8g2.drawStr(2, 35, ""); // Leer im Standard Modus
          }
        } // Scope für GELB_REST beenden
          break;
    }
    snprintf(buf, sizeof(buf), "Zustand: %s", stateTextCStr);
    u8g2.drawStr(2, 10, buf);

    // Anzeige der IP-Adresse (Immer sichtbar, wenn nicht im Menü/OTA)
    // Geändert: Nur in den ersten 10 Sekunden anzeigen
    if (millis() < 10000) {
      // WiFi.localIP().toString() returns a String. Avoid if possible.
      IPAddress ip = WiFi.localIP();
      snprintf(buf, sizeof(buf), "IP: %d.%d.%d.%d", ip[0],ip[1],ip[2],ip[3]);
      u8g2.drawStr(2, 45, buf);
    } else {
      // u8g2.drawStr(2, 45, ""); // Zeile 45 leer lassen nach 10 Sekunden
    }

    // NEU: Anzeige des Verbindungsstatus
    snprintf(buf, sizeof(buf), "Link:%s", connectionOk ? "OK" : "--");
    u8g2.drawStr(80, 60, buf);

    // Anzeige von Modus und Rolle (Immer sichtbar)
    snprintf(buf, sizeof(buf), "Modus: %s", targetMode);
    u8g2.drawStr(2, 50, buf);
    snprintf(buf, sizeof(buf), "Rolle: %s", currentRole);
    u8g2.drawStr(2, 60, buf);
  
    u8g2.sendBuffer(); // Display aktualisieren
  
    // updateLEDs() wird separat am Ende von loop() aufgerufen, wenn !isOtaRunning
  }

void drawMenu() {
  if (!inMenu) return;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
  char buf[64]; // Buffer for menu item display

  if (!inSelectionMode) {
      // --- Hauptmenü zeichnen ---
      u8g2.drawStr(2, 10, "Einstellungen:");

      // Menüpunkte mit Auswahl-Indikator und aktuellem Wert
      // Die Schleife geht jetzt bis zum letzten Element, das "Zurueck" ist
      for (int i = 0; i < sizeof(menuItems) / sizeof(menuItems[0]); i++) {
        const char* itemText = menuItems[i];
        const char* currentValue = "";
        int yPos = 25 + i * 8; // Vertikaler Abstand angepasst an Schriftart
 
        // Spezielle Behandlung für "Web Log" und andere Einstellungen
        if (strcmp(itemText, PSTR("Web Log")) == 0) { // Vergleiche mit "Web Log"
            currentValue = WebSerial.isWebLogEnabled() ? PSTR("Enabled") : PSTR("Disabled");
        } else if (strcmp(itemText, PSTR("Schiesszeit")) == 0) {
            currentValue = shootingTimeSetting;
        } else if (strcmp(itemText, PSTR("Zielmodus")) == 0) {
            currentValue = targetMode;
        } else if (strcmp(itemText, PSTR("Rolle")) == 0) {
            currentValue = currentRole;
        }
        // Für "Zurueck" bleibt currentValue leer

        if (i == currentMenuItem) {
          snprintf(buf, sizeof(buf), "> %s%s%s", itemText, (currentValue[0] != '\0' ? ": " : ""), currentValue); // Füge ": " nur hinzu, wenn currentValue nicht leer
          u8g2.drawStr(2, yPos, buf);
        } else {
          snprintf(buf, sizeof(buf), "  %s%s%s", itemText, (currentValue[0] != '\0' ? ": " : ""), currentValue); // Füge ": " nur hinzu, wenn currentValue nicht leer
          u8g2.drawStr(4, yPos, buf);
        }
      }
  } else {
      // --- Auswahlmodus zeichnen ---
      snprintf(buf, sizeof(buf), "Waehle %s:", menuItems[selectionMenuItem]);
      u8g2.drawStr(2, 10, buf);

      const char** optionsArray = nullptr; // Array von const char*
      int numOptions = 0;
      int currentActiveIndex = -1; // Index der aktuell gespeicherten Einstellung

      if (selectionMenuItem == 0) { optionsArray = shootingTimeOptions; numOptions = sizeof(shootingTimeOptions) / sizeof(shootingTimeOptions[0]); currentActiveIndex = currentShootingTimeIndex; }
      else if (selectionMenuItem == 1) { optionsArray = targetModes; numOptions = sizeof(targetModes) / sizeof(targetModes[0]); currentActiveIndex = currentTargetModeIndex; }
      else if (selectionMenuItem == 2) { optionsArray = roleModes; numOptions = sizeof(roleModes) / sizeof(roleModes[0]); currentActiveIndex = currentRoleIndex; }
      
      if (optionsArray != nullptr) {
          for (int i = 0; i < numOptions; i++) { // Schleife geht jetzt bis zum letzten Element ("Zurueck")
              int yPos = 25 + i * 8; // Vertikaler Abstand angepasst
              const char* prefix = "  "; // Standard-Einrückung
              if (i == currentSelectionIndex) {
                  prefix = "> "; // Aktuell im Auswahlmodus gewählt
              }
              String suffix = "";
              // Markiere die aktuell gespeicherte Einstellung, aber NICHT die "Zurueck"-Option
              if (i == currentActiveIndex && strcmp(optionsArray[i], PSTR("Zurueck")) != 0) {
                  suffix = " *"; 
              }
              snprintf(buf, sizeof(buf), "%s%s%s", prefix, optionsArray[i], suffix.c_str());
              u8g2.drawStr(2, yPos, buf);
          }
      }
    }

  u8g2.sendBuffer(); // Display aktualisieren
  // LEDs werden auch im Menü durch updateLEDs() am Ende von loop() aktualisiert (wenn nicht OTA).
}


// Callback-Funktion, die aufgerufen wird, wenn Daten gesendet wurden
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  WebSerial.print(F("\r\nSendestatus an: "));
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  WebSerial.print(macStr);
  WebSerial.print(F(" : "));
  WebSerial.println(status == ESP_NOW_SEND_SUCCESS ? F("Erfolgreich") : F("Fehlgeschlagen"));
}

// Callback-Funktion, die aufgerufen wird, wenn Daten empfangen werden
// Angepasste Signatur: const esp_now_recv_info_t *recv_info
// Callback-Funktion, die aufgerufen wird, wenn Daten empfangen werden
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  // Diese Zeile ist in Ordnung, da msg immer der GenericMessage-Typ ist
  GenericMessage* msg = (GenericMessage*) incomingData;

  // --- Start der IF/ELSE IF Kette ---

  // 1. Fall: MASTER empfängt PAIRING_REQUEST
  if (isMaster && msg->type == PAIRING_REQUEST && len == sizeof(PairingMessage)) {
    PairingMessage* pairMsg = (PairingMessage*) incomingData;
    WebSerial.print(F("Master: Kopplungsanfrage empfangen von: "));
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             pairMsg->macAddr[0], pairMsg->macAddr[1], pairMsg->macAddr[2], pairMsg->macAddr[3], pairMsg->macAddr[4], pairMsg->macAddr[5]);
    WebSerial.println(macStr);

    // Prüfen, ob der Peer bereits registriert ist
    bool alreadyExists = false;
    for (int i = 0; i < numRegisteredSlaves; i++) {
      if (memcmp(slaveMacs[i], pairMsg->macAddr, 6) == 0) {
        alreadyExists = true;
        break;
      }
    }

    if (!alreadyExists) {
      if (numRegisteredSlaves < MAX_SLAVES) {
        // Neuen Slave zur Liste hinzufügen
        memcpy(slaveMacs[numRegisteredSlaves], pairMsg->macAddr, 6);
        numRegisteredSlaves++; // Aktualisiere die Anzahl der registrierten Slaves
        WebSerial.println(F("Neuer Slave wird hinzugefügt und gespeichert."));
        savePeerMacsToEEPROM(); // Neue Liste im EEPROM speichern
        updateEspNowPeers();    // ESP-NOW Peer-Liste aktualisieren
      } else {
        WebSerial.println(F("Maximale Anzahl an Slaves bereits erreicht!"));
      }
    } else {
      WebSerial.println(F("Slave ist bereits bekannt."));
    }
  } // <-- End of PAIRING_REQUEST block

  // 2. Fall: SLAVE empfängt STATE_UPDATE
  else if (!isMaster && msg->type == STATE_UPDATE && len == sizeof(AmpelStateMessage)) {
    // KEIN 'if (isMaster) return;' hier, da dieser else if Block schon !isMaster prüft.

    // Korrekt auf AmpelStateMessage casten
    const AmpelStateMessage* stateMsg = reinterpret_cast<const AmpelStateMessage*>(incomingData);

    // Aktualisiere globale Zustandsvariablen direkt von der empfangenen Nachricht
    currentState = stateMsg->state;
    abCdPhase = stateMsg->abCdPhase;
    isStartingGroupAB = stateMsg->isStartingGroupAB;
    currentShootingTimeIndex = stateMsg->shootingTimeIndex;
    currentTargetModeIndex = stateMsg->targetModeIndex;
    ToneType receivedToneType = stateMsg->toneType; // Lokal deklariert und zugewiesen

    // Empfangszeit und Status aktualisieren
    lastMessageReceivedTime = millis();
    connectionOk = true;

    // Logging der empfangenen Daten
    WebSerial.print(F("Slave empfaengt Zustand: ")); WebSerial.print(currentState);
    WebSerial.print(F(" Phase: ")); WebSerial.print(abCdPhase);
    WebSerial.print(F(" Ton: ")); WebSerial.print(receivedToneType);
    WebSerial.print(F(" SchiesszeitIdx: ")); WebSerial.print(currentShootingTimeIndex);
    WebSerial.print(F(" ZielmodusIdx: ")); WebSerial.print(currentTargetModeIndex);
    WebSerial.print(F(" Startgruppe AB?: ")); WebSerial.println(isStartingGroupAB);

    // Aktualisiere die Strings basierend auf den Indices
    shootingTimeSetting = shootingTimeOptions[currentShootingTimeIndex];
    targetMode = targetModes[currentTargetModeIndex];
    updateTimings(); // Stelle sicher, dass der Slave die korrekte Grünzeit verwendet

    // Notfall-Behandlung im Slave (spezifisch für STATE_UPDATE)
    if (receivedToneType == TONE_EMERGENCY) {
      WebSerial.println(F("!!! Slave: Notfall empfangen !!!"));
      currentState = ROT; // Sofort auf ROT
      abCdPhase = 0;      // Sequenz Zuruecksetzen
      isStartingGroupAB = true;
      startTime = millis(); // Timer neu starten für ROT
      noTone(piezoPin); // Alle Töne stoppen
    }
    // Akustische Signale (Slave-Modus, spielt empfangenen Ton, wenn != TONE_NONE)
    else if (receivedToneType != TONE_NONE) { // 'else if' hier, damit nicht auch TONE_EMERGENCY abgespielt wird
      WebSerial.print("Slave: Spiele empfangenen Ton Typ: "); WebSerial.println(receivedToneType);
      noTone(piezoPin); // Alten Ton stoppen

      // Tonfolgen replizieren (Achtung: delays in Callbacks können problematisch sein!)
      switch(receivedToneType) {
        case TONE_GO_TO_LINE: // Zweimaliges Pfeifen
            tone(piezoPin, 1000, 50); delay(150); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin); break;
        case TONE_SHOOT_START: // Einmaliges Pfeifen
            tone(piezoPin, 1000, 100); noTone(piezoPin); break;
        case TONE_GROUP_CHANGE: // Zweimaliges Pfeifen
            tone(piezoPin, 1000, 50); delay(150); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin); break;
        case TONE_END_OF_END: // Dreimaliges Pfeifen
            tone(piezoPin, 1000, 50); delay(150); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin); delay(50); tone(piezoPin, 1000, 50); noTone(piezoPin); break;
        case TONE_EMERGENCY: // Wird bereits oben behandelt, hier nichts mehr tun
            WebSerial.println(F("Slave: Notfallton wird nicht erneut gespielt."));
            break;
        default: noTone(piezoPin); break;
      }
    } else {
      noTone(piezoPin); // Stoppe jeden laufenden Ton
    }
  } // <-- End of STATE_UPDATE block

  // 3. Fall: SLAVE empfängt PING (Keep-Alive)
  else if (!isMaster && msg->type == PING && len == sizeof(KeepAliveMsg)) { // Zeile 1362 (aus Ihrer Perspektive)
    lastMessageReceivedTime = millis(); // PING zählt auch als empfangene Nachricht
    connectionOk = true;

    WebSerial.println(F("Slave: PING empfangen, sende PONG zurück an Master."));

    // PONG Nachricht erstellen und an den Absender (Master) zurücksenden
    const KeepAliveMsg* keepAliveMsg = reinterpret_cast<const KeepAliveMsg*>(incomingData);
    KeepAliveMsg pongMsg = {PONG};
    esp_now_send(recv_info->src_addr, (uint8_t *) &pongMsg, sizeof(pongMsg));
  } // <-- End of PING block

  // 4. Fall: MASTER empfängt PONG (Keep-Alive Antwort)
  else if (isMaster && msg->type == PONG && len == sizeof(KeepAliveMsg)) { // Zeile 1397 (aus Ihrer Perspektive)
    // Finden Sie den Slave-Index anhand der MAC-Adresse
    int slaveIndex = -1;
    for (int i = 0; i < numRegisteredSlaves; ++i) {
        if (memcmp(slaveMacs[i], recv_info->src_addr, 6) == 0) {
            slaveIndex = i;
            break;
        }
    }
    if (slaveIndex != -1) {
        lastPongReceivedTime[slaveIndex] = millis(); // Aktualisieren für den spezifischen Slave
        // connectionOk = true; // Dies müsste für jeden Slave einzeln verfolgt werden, wenn mehrere Slaves vorhanden sind
    }

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    WebSerial.printf("Master: PONG empfangen von %s\n", macStr);
  } // <-- End of PONG block

  // 5. Fall: Unbekannter Nachrichtentyp oder falsche Länge
  else { // Zeile 1440 (aus Ihrer Perspektive)
    WebSerial.print(F("Unbekannter Nachrichtentyp oder Länge: Typ="));
    WebSerial.print(msg->type);
    WebSerial.print(F(", Länge="));
    WebSerial.print(len);
    WebSerial.println();
  }
} // <-- Die letzte schließende Klammer der Funktion OnDataRecv (Zeile 1443)

// Webserver-Handler für die Root-Seite
void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", ""); // Send headers

  server.sendContent_P(PSTR("<html><head><title>Bogenampel</title></head><body>"));
  server.sendContent_P(PSTR("<h1>Bogenschie&szlig;ampel Status</h1>"));

  // Zustand und prominente Zeitanzeige
  server.sendContent_P(PSTR("<p><b>Zustand: "));
  const char* stateTextWebCStr;
  const char* stateColorCStr;
   switch (currentState) {
    case ROT:       stateTextWebCStr = "ROT"; stateColorCStr = "red"; break;
    case GELB_PREP: stateTextWebCStr = "GELB (Vorbereitung)"; stateColorCStr = "orange"; break;
    case GRUEN:     stateTextWebCStr = "GRUEN"; stateColorCStr = "green"; break;
    case GELB_REST: stateTextWebCStr = "GELB (Restzeit)"; stateColorCStr = "orange"; break;
    default:        stateTextWebCStr = "Unbekannt"; stateColorCStr = "grey"; break;
   }
  
  char buf[128]; // Buffer for HTML parts
  // Platzhalter für Zustand (wird durch JS ersetzt) - Füge ID hinzu
  snprintf(buf, sizeof(buf), "<span id='stateText'><font color='%s'>%s</font></span>", stateColorCStr, stateTextWebCStr);
  server.sendContent(buf);
  server.sendContent_P(PSTR("</b></p>"));

  if (currentState == GELB_PREP || currentState == GRUEN || currentState == GELB_REST) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;
    unsigned long totalDurationForTime = 0;
    if(currentState == GELB_PREP) totalDurationForTime = yellowDurationPrep;
    else if (currentState == GRUEN) totalDurationForTime = currentGreenDuration;
    else if (currentState == GELB_REST) totalDurationForTime = yellowDurationRest;
    unsigned long remainingTimeMs = 0;
     if (totalDurationForTime > elapsedTime) {
          remainingTimeMs = totalDurationForTime - elapsedTime;
      } else {
          remainingTimeMs = 0;
      }
    int remainingSecondsTotal = remainingTimeMs / 1000;
    int minutes = remainingSecondsTotal / 60;
    int seconds = remainingSecondsTotal % 60;
    
    // Platzhalter für Zeit (wird durch JS ersetzt) - Füge ID hinzu
    snprintf(buf, sizeof(buf), "<h2>Verbleibende Zeit: <span id='remainingTime'>%02d:%02d</span></h2>", minutes, seconds);
    server.sendContent(buf);
  } else {
      // Platzhalter für Zeit (wird durch JS ersetzt) - Füge ID hinzu
      server.sendContent_P(PSTR("<h2>Verbleibende Zeit: <span id='remainingTime'>Warten...</span></h2>"));
  }

  server.sendContent_P(PSTR("<p>Aktuelle Einstellungen:</p><ul>"));
  snprintf(buf, sizeof(buf), "<li>Schie&szlig;zeit: <span id='shootingTimeSetting'>%s</span></li>", shootingTimeSetting);
  server.sendContent(buf);
  snprintf(buf, sizeof(buf), "<li>Zielmodus: <span id='targetModeSetting'>%s</span></li>", targetMode);
  server.sendContent(buf);

  if (targetMode == targetModes[1]) { // "AB/CD"
     snprintf(buf, sizeof(buf), "<li>Aktuelle Gruppe: <span id='currentGroup'>%s</span></li>", (abCdPhase == 0 ? "AB" : "CD"));
     server.sendContent(buf);
     snprintf(buf, sizeof(buf), "<li>Startgruppe n&auml;chste Passe: <span id='nextStartGroup'>%s</span></li>", (isStartingGroupAB ? "AB" : "CD"));
     server.sendContent(buf);
  }
  snprintf(buf, sizeof(buf), "<li>Rolle: <span id='roleSetting'>%s</span></li>", currentRole);
  server.sendContent(buf);
  
  IPAddress ip = WiFi.localIP();
  snprintf(buf, sizeof(buf), "<li>IP: %d.%d.%d.%d</li>", ip[0],ip[1],ip[2],ip[3]);
  server.sendContent(buf);

  snprintf(buf, sizeof(buf), "<li>Eigene MAC: %s</li>", WiFi.macAddress().c_str()); // macAddress returns String
  server.sendContent(buf);

  snprintf(buf, sizeof(buf), "<li>Konfigurierte Slaves: <span id='numSlavesDisplay'>%d</span></li>", numRegisteredSlaves);
  server.sendContent(buf);

  if (numRegisteredSlaves > 0) {
      snprintf(buf, sizeof(buf), "<li>Erste Slave MAC: <span id='firstSlaveMacDisplay'>%02X:%02X:%02X:%02X:%02X:%02X</span></li>", 
               slaveMacs[0][0], slaveMacs[0][1], slaveMacs[0][2], slaveMacs[0][3], slaveMacs[0][4], slaveMacs[0][5]);
      server.sendContent(buf);
  }

  // Füge ID für Link-Status hinzu
  snprintf(buf, sizeof(buf), "<li>Verbindung zum Peer: <span id='linkStatus'>%s</span></li>", (connectionOk ? "OK" : "--"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("</ul>"));

  // Steuerung und Einstellungen (nur im Master-Modus und wenn Geraet nicht beschaeftigt ist)
  snprintf(buf, sizeof(buf), "<div id='masterControlSection' style='display: %s;'>", (isMaster && !isOtaRunning && !otaTimedOut ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<h2>Steuerung (Master)</h2>"));
  snprintf(buf, sizeof(buf), "<div id='startBtnContainer' style='display: %s;'>", (isMaster && currentState == ROT && !isOtaRunning && !otaTimedOut ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<p><a href='/start'><button>START Passe</button></a></p></div>"));
  snprintf(buf, sizeof(buf), "<div id='stopBtnContainer' style='display: %s;'>", (isMaster && (currentState == GELB_PREP || currentState == GRUEN || currentState == GELB_REST) && !isOtaRunning && !otaTimedOut ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<p><a href='/stop'><button>STOP Passe</button></a></p></div>"));
  snprintf(buf, sizeof(buf), "<div id='resetBtnContainer' style='display: %s;'>", (isMaster && !isOtaRunning && !otaTimedOut ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<p><a href='/reset'><button>Reset Ampel</button></a></p></div>"));
  server.sendContent_P(PSTR("</div>")); // Ende masterControlSection

  // Container für Not-Stop (immer sichtbar außer bei OTA)
  snprintf(buf, sizeof(buf), "<div id='emergencyBtnContainer' style='display: %s;'>", (!isOtaRunning && !otaTimedOut ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<p><a href='/emergencyStop'><button style='background-color:red; color:white;'>NOT-STOP</button></a></p></div>"));

  // Container für Master-Einstellungen
  snprintf(buf, sizeof(buf), "<div id='masterSettingsSection' style='display: %s;'>", (isMaster && !isOtaRunning && !otaTimedOut ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<h2>Einstellungen &auml;ndern (Master)</h2>"));
  snprintf(buf, sizeof(buf), "<p>Schie&szlig;zeit aktuell: <span id='shootingTimeSettingDisplay'>%s</span> ", shootingTimeSetting);
  server.sendContent(buf);
  server.sendContent_P(PSTR("<a href='/setSchiesszeit?time=3'><button>Setze auf 3 Pfeile</button></a> "));
  server.sendContent_P(PSTR("<a href='/setSchiesszeit?time=6'><button>Setze auf 6 Pfeile</button></a></p>"));
  snprintf(buf, sizeof(buf), "<p>Zielmodus aktuell: <span id='targetModeSettingDisplay'>%s</span> ", targetMode);
  server.sendContent(buf);
  server.sendContent_P(PSTR("<a href='/setZielmodus?mode=Standard'><button>Setze auf Standard</button></a> "));
  server.sendContent_P(PSTR("<a href='/setZielmodus?mode=ABCD'><button>Setze auf AB/CD</button></a></p>"));
  server.sendContent_P(PSTR("</div>")); // Ende masterSettingsSection

  // Container für Peer MAC Form (immer sichtbar außer bei OTA)
  snprintf(buf, sizeof(buf), "<div id='peerMacFormContainer' style='display: %s;'>", (!isOtaRunning && !otaTimedOut ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<h2>Geraete-Konfiguration</h2>"));
  snprintf(buf, sizeof(buf), "<p>Rolle aktuell: <span id='roleSettingDisplay'>%s</span> ", currentRole);
  server.sendContent(buf);
  server.sendContent_P(PSTR("<a href='/setRolle?role=Master'><button>Setze auf Master</button></a> "));
  server.sendContent_P(PSTR("<a href='/setRolle?role=Slave'><button>Setze auf Slave</button></a></p>"));
  server.sendContent_P(PSTR("<h2>ESP-NOW Peer Konfiguration</h2><form action='/setPeerMac' method='get'>"));

     for (int i = 0; i < MAX_SLAVES; ++i) {
        snprintf(buf, sizeof(buf), "<p>Slave %d MAC: ", i + 1);
        server.sendContent(buf);
        char macInputName[10]; // "mac" + index + null -> Größe angepasst
        sprintf(macInputName, "mac%d", i);
        char currentMacStrWeb[18];
        snprintf(currentMacStrWeb, sizeof(currentMacStrWeb), "%02X:%02X:%02X:%02X:%02X:%02X", slaveMacs[i][0], slaveMacs[i][1], slaveMacs[i][2], slaveMacs[i][3], slaveMacs[i][4], slaveMacs[i][5]);
        snprintf(buf, sizeof(buf), "<input type='text' name='%s' placeholder='XX:XX:XX:XX:XX:XX' value='%s'></p>", macInputName, currentMacStrWeb);
        server.sendContent(buf);
     }
  server.sendContent_P(PSTR("<input type='submit' value='Speichern'></form></div>"));

  // Container für OTA Link (nur sichtbar wenn Master nicht busy)
  snprintf(buf, sizeof(buf), "<div id='otaLinkContainer' style='display: %s;'>", (isMaster && !isOtaRunning && !otaTimedOut ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<h2>Firmware Update (OTA)</h2>"));
  server.sendContent_P(PSTR("<p>Firmware &uuml;ber Webinterface aktualisieren: <a href='/update'><button>Firmware hochladen</button></a></p>"));
  server.sendContent_P(PSTR("<p><small>(Navigiert zu '/update', wo Sie die .bin Datei w&auml;hlen k&ouml;nnen)</small></p></div>"));

  // Container für Slave-Hinweis
  snprintf(buf, sizeof(buf), "<div id='slaveInfoSection' style='display: %s;'>", (!isMaster ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<h2>Steuerung & Einstellungen (Slave)</h2>"));
  server.sendContent_P(PSTR("<p>Steuerung und Einstellungen erfolgen &uuml;ber das Webinterface des Master-Ger&auml;ts.</p></div>"));

  // Container für Busy-Hinweis (OTA)
  snprintf(buf, sizeof(buf), "<div id='busyInfoSection' style='display: %s;'>", (isOtaRunning || otaTimedOut ? "block" : "none"));
  server.sendContent(buf);
  server.sendContent_P(PSTR("<h2>Ger&auml;t besch&auml;ftigt</h2>"));
  server.sendContent_P(PSTR("<p>OTA Update l&auml;uft oder ist fehlgeschlagen. Einstellungen & Steuerung sind deaktiviert.</p></div>"));

  // NEU: Abschnitt für Serial Log
  server.sendContent_P(PSTR("<div id='serialLogSection'><h2>Serielles Debug Log (Web)</h2><p><button onclick=\"toggleLog()\">"));
  server.sendContent(WebSerial.isWebLogEnabled() ? PSTR("Web-Log Deaktivieren") : PSTR("Web-Log Aktivieren"));
  server.sendContent_P(PSTR("</button> <button onclick=\"clearLog()\">Log Leeren</button></p>"));
  server.sendContent_P(PSTR("<textarea id='serialLogOutput' readonly rows='15' style='width: 95%; font-family: monospace; font-size: 0.8em;'></textarea></div>"));

  // --- JavaScript für dynamische Updates ---
  server.sendContent_P(PSTR(R"rawliteral(
<script>
  function fetchStatus() {
    fetch('/status')
      .then(response => {
        if (!response.ok) {
          throw new Error('Network response was not ok ' + response.statusText);
        }
        return response.json();
      })
      .then(data => {
        // Update Status Text und Farbe
        const stateElement = document.getElementById('stateText');
        if (stateElement) {
           stateElement.innerHTML = '<font color="' + data.stateColor + '">' + data.state + '</font>'; // Verwende innerHTML für <font> etc.
        }

        // Update verbleibende Zeit
        const timeElement = document.getElementById('remainingTime');
        if (timeElement) {
           timeElement.textContent = data.remainingTimeStr;
        }

        // Update Einstellungen
        const shootingTimeElement = document.getElementById('shootingTimeSetting');
        if (shootingTimeElement) shootingTimeElement.textContent = data.shootingTime;
        const targetModeElement = document.getElementById('targetModeSetting');
        if (targetModeElement) targetModeElement.textContent = data.targetMode;
        const roleElement = document.getElementById('roleSetting');
        if (roleElement) roleElement.textContent = data.role;
        // Aktualisiere die Anzeige der Anzahl und der ersten MAC
        const numSlavesDisp = document.getElementById('numSlavesDisplay');
        if (numSlavesDisp) numSlavesDisp.textContent = data.numSlaves;
        const firstMacDisp = document.getElementById('firstSlaveMacDisplay');
        if (firstMacDisp) firstMacDisp.textContent = data.firstPeerMac; // Neues Feld im JSON

        const linkStatusElement = document.getElementById('linkStatus');
        if (linkStatusElement) {
            linkStatusElement.textContent = data.linkStatus;
            linkStatusElement.style.color = (data.linkStatus === 'OK') ? 'green' : 'red';
        }

        // Update AB/CD Infos (nur wenn Elemente existieren)
        const currentGroupElement = document.getElementById('currentGroup');
        if (currentGroupElement) currentGroupElement.textContent = data.currentGroup;
        const nextStartGroupElement = document.getElementById('nextStartGroup');
        if (nextStartGroupElement) nextStartGroupElement.textContent = data.nextStartGroup;

        // Update Sichtbarkeit der Sektionen/Buttons
        document.getElementById('masterControlSection').style.display = (data.isMaster && !data.otaRunning && !data.otaTimeout) ? 'block' : 'none';
        document.getElementById('startBtnContainer').style.display = data.showStartButton ? 'block' : 'none';
        document.getElementById('stopBtnContainer').style.display = data.showStopButton ? 'block' : 'none';
        document.getElementById('resetBtnContainer').style.display = data.showResetButton ? 'block' : 'none';
        document.getElementById('emergencyBtnContainer').style.display = data.showEmergencyButton ? 'block' : 'none';
        document.getElementById('masterSettingsSection').style.display = data.showSettings ? 'block' : 'none';
        document.getElementById('peerMacFormContainer').style.display = data.showPeerMacForm ? 'block' : 'none';
        document.getElementById('otaLinkContainer').style.display = data.showOtaLink ? 'block' : 'none';
        document.getElementById('slaveInfoSection').style.display = (!data.isMaster) ? 'block' : 'none';
        document.getElementById('busyInfoSection').style.display = (data.otaRunning || data.otaTimeout) ? 'block' : 'none';

        // Update der Button-Anzeigen in den Einstellungen (optional, aber konsistent)
        const shootingTimeDisplay = document.getElementById('shootingTimeSettingDisplay');
        if (shootingTimeDisplay) shootingTimeDisplay.textContent = data.shootingTime;
        const targetModeDisplay = document.getElementById('targetModeSettingDisplay');
        if (targetModeDisplay) targetModeDisplay.textContent = data.targetMode;
        const roleDisplay = document.getElementById('roleSettingDisplay');
        if (roleDisplay) roleDisplay.textContent = data.role;

        // Update Serial Log Button Text
        const toggleButton = document.querySelector('#serialLogSection button:first-of-type');
        if (toggleButton) {
            toggleButton.textContent = data.webSerialLogEnabled ? "Web-Log Deaktivieren" : "Web-Log Aktivieren";
        }
      })
      .catch(error => {
        console.error('Error fetching status:', error);
        // Optional: Zeige einen Fehler auf der Seite an
        const stateElement = document.getElementById('stateText');
        if (stateElement) {
           stateElement.innerHTML = "<font color='red'>Verbindung verloren</font>";
        }
        const linkStatusElement = document.getElementById('linkStatus');
         if (linkStatusElement) {
            linkStatusElement.textContent = '??';
            linkStatusElement.style.color = 'red';
        }
      });
  }

  function fetchSerialLogContent() {
    if (!document.getElementById('serialLogOutput')) return; // Element nicht da
    // Nur abrufen, wenn das Log-Fenster sichtbar ist oder der Button "aktiviert" anzeigt
    // Die Logik, ob geloggt wird, ist serverseitig. Hier holen wir nur, was da ist.
    fetch('/getSerialLog')
      .then(response => response.text())
      .then(text => {
        const logArea = document.getElementById('serialLogOutput');
        logArea.value = text;
        logArea.scrollTop = logArea.scrollHeight; // Auto-scroll
      })
      .catch(error => console.error('Error fetching serial log:', error));
  }

  function toggleLog() {
    fetch('/toggleWebSerialLog').then(() => fetchStatus()); // Status neu laden, um Button-Text zu aktualisieren
  }

  function clearLog() {
    fetch('/clearWebSerialLog').then(() => fetchSerialLogContent()); // Log-Inhalt neu laden (sollte leer sein)
  }

  // Rufe fetchStatus alle 1 Sekunde auf
  setInterval(fetchStatus, 1000);
  // Rufe fetchSerialLogContent alle 2 Sekunden auf (weniger häufig als Status)
  setInterval(fetchSerialLogContent, 2000);

  // Rufe fetchStatus einmal direkt beim Laden auf
  document.addEventListener('DOMContentLoaded', fetchStatus);
  // Rufe fetchSerialLogContent einmal direkt beim Laden auf
  document.addEventListener('DOMContentLoaded', fetchSerialLogContent);

</script>
)rawliteral") );

  server.sendContent_P(PSTR("</body></html>"));
  server.sendContent(""); // Finalize
}



// NEU: Handler, der den aktuellen Status als JSON Zurueckgibt
void handleStatusJson() {
  StaticJsonDocument<1024> doc; // Größe anpassen, falls mehr Daten benötigt werden

  // Aktuellen Zustand und Farbe bestimmen
  const char* stateTextWebCStr;
  const char* stateColorCStr;
  switch (currentState) {
    case ROT:       stateTextWebCStr = "ROT"; stateColorCStr = "red"; break;
    case GELB_PREP: stateTextWebCStr = "GELB (Vorb.)"; stateColorCStr = "orange"; break;
    case GRUEN:     stateTextWebCStr = "GRUEN"; stateColorCStr = "green"; break;
    case GELB_REST: stateTextWebCStr = "GELB (Rest)"; stateColorCStr = "orange"; break;
    default:        stateTextWebCStr = "Unbekannt"; stateColorCStr = "grey"; break;
  }
  if (isOtaRunning) { stateTextWebCStr = "OTA Update..."; stateColorCStr = "blue"; }
  else if (otaTimedOut) { stateTextWebCStr = "OTA Timeout!"; stateColorCStr = "red"; }

  doc["state"] = stateTextWebCStr;
  doc["stateColor"] = stateColorCStr;

  // Verbleibende Zeit berechnen und formatieren
  char remainingTimeStrBuf[10] = "Warten..."; // Buffer for time string
  if (currentState == GELB_PREP || currentState == GRUEN || currentState == GELB_REST) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;
    unsigned long totalDurationForTime = 0;
    if(currentState == GELB_PREP) totalDurationForTime = yellowDurationPrep;
    else if (currentState == GRUEN) totalDurationForTime = currentGreenDuration;
    else if (currentState == GELB_REST) totalDurationForTime = yellowDurationRest;
    unsigned long remainingTimeMs = 0;
    if (totalDurationForTime > elapsedTime) {
      remainingTimeMs = totalDurationForTime - elapsedTime;
    }
    int remainingSecondsTotal = remainingTimeMs / 1000;
    int minutes = remainingSecondsTotal / 60;
    int seconds = remainingSecondsTotal % 60;
    snprintf(remainingTimeStrBuf, sizeof(remainingTimeStrBuf), "%02d:%02d", minutes, seconds);
  }
  doc["remainingTimeStr"] = remainingTimeStrBuf;

  // Einstellungen und andere Status
  doc["shootingTime"] = shootingTimeSetting;
  doc["targetMode"] = targetMode;
  doc["role"] = currentRole;
  doc["ip"] = WiFi.localIP().toString();
  doc["myMac"] = WiFi.macAddress();

  // Sende die erste Slave MAC (falls vorhanden) und die Anzahl
  char firstSlaveMacJson[18] = "--:--:--:--:--:--";
  if (numRegisteredSlaves > 0) snprintf(firstSlaveMacJson, sizeof(firstSlaveMacJson), "%02X:%02X:%02X:%02X:%02X:%02X", slaveMacs[0][0], slaveMacs[0][1], slaveMacs[0][2], slaveMacs[0][3], slaveMacs[0][4], slaveMacs[0][5]);
  doc["firstPeerMac"] = firstSlaveMacJson;
  doc["numSlaves"] = numRegisteredSlaves; // Anzahl der registrierten Slaves hinzufügen

  doc["linkStatus"] = connectionOk ? "OK" : "--";
  doc["isMaster"] = isMaster;
  doc["otaRunning"] = isOtaRunning;
  doc["otaTimeout"] = otaTimedOut;

  // AB/CD spezifische Infos
  if (targetMode == targetModes[1]) { // "AB/CD"
    doc["currentGroup"] = (currentState != ROT) ? (abCdPhase == 0 ? "AB" : "CD") : ""; // Nur anzeigen wenn aktiv
    doc["nextStartGroup"] = isStartingGroupAB ? "AB" : "CD";
  } else {
    doc["currentGroup"] = "";
    doc["nextStartGroup"] = "";
  }
  // Flags für die Anzeige von Buttons/Sektionen im Frontend
  bool canControl = isMaster && !isOtaRunning && !otaTimedOut;
  doc["showStartButton"] = canControl && currentState == ROT;
  doc["showStopButton"] = canControl && (currentState == GELB_PREP || currentState == GRUEN || currentState == GELB_REST);
  doc["showResetButton"] = canControl;
  doc["showEmergencyButton"] = !isOtaRunning && !otaTimedOut; // Notstopp immer, außer bei OTA
  doc["showSettings"] = canControl; // Master-spezifische Einstellungen (Schießzeit, Zielmodus)
  doc["showPeerMacForm"] = !isOtaRunning && !otaTimedOut; // Peer MAC immer, außer bei OTA
  doc["showOtaLink"] = canControl; // OTA Link nur wenn Master nicht busy
  doc["webSerialLogEnabled"] = WebSerial.isWebLogEnabled(); // NEU

  // JSON senden
  String jsonOutput;
  serializeJson(doc, jsonOutput);
  server.send(200, "application/json", jsonOutput);
}


// Webserver-Handler zum Starten der Passe (setzt manualStartTriggered Flag)
void handleWebStart() {
  // Start ist nur aus dem ROT Zustand moeglich und nur im Master Modus
  if (isMaster && currentState == ROT && !isOtaRunning && !otaTimedOut) {
    manualStartTriggered = true; // Setze das Start-Flag
    WebSerial.println(F("Web: Start Trigger gesetzt"));
    server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
    server.send(302, "text/plain", "");       // Sende Redirect-Status
  } else if (!isMaster) {
    server.send(403, "text/plain", "Forbidden: Not in Master mode.");
  } else if (currentState != ROT) {
     server.send(409, "text/plain", "Conflict: Cannot start from current state (not in ROT).");
  } else { // OTA läuft oder Timeout
     server.send(503, "text/plain", "Service Unavailable: Device busy (OTA).");
  }
}

// Webserver-Handler zum Stoppen der Passe (setzt manualStopTriggered Flag)
void handleWebStop() {
  // Stoppen ist aus den aktiven Phasen moeglich (GELB_PREP, GRUEN, GELB_REST) und nur im Master Modus
  if (isMaster && (currentState == GELB_PREP || currentState == GRUEN || currentState == GELB_REST) && !isOtaRunning && !otaTimedOut) {
    manualStopTriggered = true; // Setze das Stop-Flag
    WebSerial.println(F("Web: Stop Trigger gesetzt"));
    server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
    server.send(302, "text/plain", "");       // Sende Redirect-Status
  } else if (!isMaster) {
    server.send(403, "text/plain", "Forbidden: Not in Master mode.");
  } else if (currentState == ROT) {
     server.send(409, "text/plain", "Conflict: Already in ROT state.");
  } else { // OTA läuft oder Timeout
     server.send(503, "text/plain", "Service Unavailable: Device busy (OTA).");
  }
}

// Webserver-Handler zum Zuruecksetzen der Ampel auf Anfang (ROT, AB als Startgruppe)
void handleWebReset() {
   if (isMaster && !isOtaRunning && !otaTimedOut) {
       WebSerial.println(F("Web: Reset Trigger erhalten. Setze Ampel zurueck."));
       abCdPhase = 0; // Immer mit AB Phase 0 starten
       isStartingGroupAB = true; // Immer mit AB als Startgruppe starten
       setState(ROT, TONE_NONE); // Zustand auf ROT setzen, kein Ton
       server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
       server.send(302, "text/plain", "");       // Sende Redirect-Status
   } else if (!isMaster) {
       server.send(403, "text/plain", "Forbidden: Only Master can reset.");
   } else { // OTA läuft oder Timeout
       server.send(503, "text/plain", "Service Unavailable: Device busy (OTA).");
   }
}

// Web-Handler für Notstopp
void handleEmergencyStop() {
  // Notstopp sollte immer möglich sein, außer OTA läuft
  if (!isOtaRunning && !otaTimedOut) {
    triggerEmergencyStop(); // Rufe die zentrale Notstopp-Funktion auf
    server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
    server.send(302, "text/plain", "");       // Sende Redirect-Status
  } else {
    server.send(503, "text/plain", "Service Unavailable: Device busy (OTA).");
  }
}


// Webserver-Handler zum Setzen der Schießzeit (3 oder 6 Pfeile)
void handleSetSchiesszeit() {
  if (!isMaster) {
    server.send(403, "text/plain", "Forbidden: Only Master can change settings.");
    return;
  }

  if (server.hasArg("time")) {
    // server.arg("time") returns a String. Use .equals() for comparison.
    if (server.arg("time").equals("3")) {
      shootingTimeSetting = shootingTimeOptions[0]; // "3 Pfeile"
      currentShootingTimeIndex = 0;
      updateTimings();
      printCurrentSettings();
      server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
      server.send(302, "text/plain", "");       // Sende Redirect-Status
    } else if (server.arg("time").equals("6")) {
      shootingTimeSetting = shootingTimeOptions[1]; // "6 Pfeile"
      currentShootingTimeIndex = 1;
      updateTimings();
      printCurrentSettings();
      server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
      server.send(302, "text/plain", "");       // Sende Redirect-Status
    } else {
      server.send(400, "text/plain", "Bad Request: Invalid time parameter. Use '3' or '6'.");
    }
  } else {
    server.send(400, "text/plain", "Bad Request: Missing 'time' parameter.");
  }
}

// Webserver-Handler zum Setzen des Zielmodus (Standard oder AB/CD)
void handleSetZielmodus() {
  if (!isMaster) {
    server.send(403, "text/plain", "Forbidden: Only Master can change settings.");
    return;
  }

  if (server.hasArg("mode")) {
    if (server.arg("mode").equals("Standard")) {
      targetMode = targetModes[0]; // "Standard"
      currentTargetModeIndex = 0;
      printCurrentSettings();
       // Beim Wechsel des Zielmodus im Master, Zuruecksetzen auf Anfang Standard
       abCdPhase = 0; // Phase Zuruecksetzen
       isStartingGroupAB = true; // Startgruppe Zuruecksetzen
       setState(ROT, TONE_NONE); // Zurueck zu ROT, kein Ton
      server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
      server.send(302, "text/plain", "");       // Sende Redirect-Status
    } else if (server.arg("mode").equals("ABCD")) {
      targetMode = targetModes[1]; // "AB/CD"
      currentTargetModeIndex = 1;
      printCurrentSettings();
       // Beim Wechsel des Zielmodus im Master, Zuruecksetzen auf Anfang AB/CD (AB startet)
       abCdPhase = 0; // Phase Zuruecksetzen
       isStartingGroupAB = true; // Startgruppe Zuruecksetzen
       setState(ROT, TONE_NONE); // Zurueck zu ROT, kein Ton
      server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
      server.send(302, "text/plain", "");       // Sende Redirect-Status
    } else {
      server.send(400, "text/plain", "Bad Request: Invalid mode parameter. Use 'Standard' or 'ABCD'.");
    }
  } else {
    server.send(400, "text/plain", "Bad Request: Missing 'mode' parameter.");
  }
}

// Webserver-Handler zum Setzen der Rolle (Master oder Slave)
void handleSetRolle() {
  if (server.hasArg("role")) {
    if (server.arg("role").equals("Master")) {
      currentRole = roleModes[0]; // "Master"
      currentRoleIndex = 0;
      bool oldIsMaster = isMaster;
      isMaster = true;
      printCurrentSettings();
       if (!oldIsMaster) { // Wenn vorher Slave war
         WebSerial.println(F("Rolle auf Master gesetzt. ESP-NOW senden aktiv."));
         // Als neuer Master sollten wir den Zustand auf ROT setzen und senden (Anfangszustand)
         abCdPhase = 0; // Phase Zuruecksetzen
         isStartingGroupAB = true; // Startgruppe Zuruecksetzen
         setState(ROT, TONE_NONE); // Setze Zustand auf ROT und sende via ESP-NOW
       }
      saveRoleToEEPROM(); // Rolle immer speichern, wenn sie explizit über das Webinterface gesetzt wird
      server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
      server.send(302, "text/plain", "");       // Sende Redirect-Status
    } else if (server.arg("role").equals("Slave")) {
      currentRole = roleModes[1]; // "Slave"
      currentRoleIndex = 1;
      bool oldIsMaster = isMaster;
      isMaster = false;
      printCurrentSettings();
      if (oldIsMaster) { // Wenn vorher Master war
        WebSerial.println(F("Rolle auf Slave gesetzt. ESP-NOW empfangen aktiv."));
        // Als Slave lokalen Zustand beibehalten, bis NRF kommt.
      }
      saveRoleToEEPROM(); // Rolle immer speichern, wenn sie explizit über das Webinterface gesetzt wird
      server.sendHeader("Location", "/", true); // Redirect zur Hauptseite
      server.send(302, "text/plain", "");       // Sende Redirect-Status
    } else {
      server.send(400, "text/plain", "Bad Request: Invalid role parameter. Use 'Master' or 'Slave'.");
    }
  } else {
    server.send(400, "text/plain", "Bad Request: Missing 'role' parameter.");
  }
}

// Webserver-Handler zum Setzen der Peer MAC Adresse
void handleSetPeerMac() {
  // Erlauben wir beiden Rollen, die Peer MAC zu setzen? Ja, das ist sinnvoll.

  WebSerial.println(F("Web: Empfange neue Slave MAC Adressen..."));
  bool parseError = false;

  // Temporäres Array zum Speichern der alten MACs für die Peer-Entfernung
  // (Nicht unbedingt nötig, da updateEspNowPeers alle entfernt, aber schadet nicht)
  // uint8_t oldSlaveMacs[MAX_SLAVES][6];
  // memcpy(oldSlaveMacs, slaveMacs, sizeof(slaveMacs));

  // Iteriere durch die erwarteten Argumente mac0, mac1, ...
  for (int i = 0; i < MAX_SLAVES; ++i) {
    char argName[6]; // "mac" + index + null
    sprintf(argName, "mac%d", i);

    if (server.hasArg(argName)) {
      String macStr = server.arg(argName); // Keep as String for sscanf
      uint8_t parsedMac[6];

      // Versuche, die MAC-Adresse zu parsen
      if (sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &parsedMac[0], &parsedMac[1], &parsedMac[2], &parsedMac[3], &parsedMac[4], &parsedMac[5]) == 6) {
        // Erfolgreich geparsed, übernehme in das globale Array
        memcpy(slaveMacs[i], parsedMac, 6);
        WebSerial.printf("  Slave %d MAC gesetzt auf: %s\n", i + 1, macStr.c_str());
      } else {
        // Ungültiges Format oder leeres Feld
        // Setze die MAC für diesen Index auf "ungültig" (z.B. FF:FF:FF:FF:FF:FF)
        memset(slaveMacs[i], 0xFF, 6);
        WebSerial.printf("  Slave %d MAC ungültig oder leer ('%s'), setze auf FF:FF:FF:FF:FF:FF\n", i + 1, macStr.c_str());
        // Optional: Fehlerflag setzen, aber wir speichern trotzdem die "leeren" Einträge
        // parseError = true;
      }
    } else {
      // Argument nicht gefunden - das sollte nicht passieren, wenn das Formular korrekt ist.
      // Behalte den alten Wert oder setze auf ungültig? Sicherer ist ungültig.
      memset(slaveMacs[i], 0xFF, 6);
      WebSerial.printf("  Slave %d MAC Argument '%s' nicht gefunden, setze auf FF:FF:FF:FF:FF:FF\n", i + 1, argName);
    }
  }

  // Speichere das aktualisierte Array im EEPROM
  savePeerMacsToEEPROM();

  // Aktualisiere die ESP-NOW Peer Liste
  updateEspNowPeers(); // Diese Funktion entfernt alte und fügt neue hinzu

  if (parseError) {
     server.send(400, "text/plain", "Warnung: Mindestens eine MAC Adresse hatte ein ungültiges Format und wurde ignoriert/Zurueckgesetzt. Andere wurden gespeichert.");
  } else {
     // Bei Erfolg zur Hauptseite Zurueckleiten
     server.sendHeader("Location", "/", true);
     server.send(302, "text/plain", "");       // Sende Redirect-Status
  }
}

// Helper-Funktion zum Aktualisieren der ESP-NOW Peers basierend auf slaveMacs
void updateEspNowPeers() {
  WebSerial.println(F("Aktualisiere ESP-NOW Peers... [START]"));

  // 1. Zuerst alle vorhandenen Peer MACs sammeln
  std::vector<std::array<uint8_t, 6>> existingPeers;
  esp_now_peer_info_t tempPeer;
  // ESP32 SDK >= 4.x: esp_now_get_peer
  // ESP32 SDK < 4.x: esp_now_fetch_peer
  // Wir verwenden hier die ältere Variante für breitere Kompatibilität
  const int MAX_FETCH_ATTEMPTS = 30; // ESP-NOW typically supports ~20 peers. Add a safety margin.

  WebSerial.println(F("  Fetching first peer..."));
  bool hasPeer = esp_now_fetch_peer(true, &tempPeer); // Get first peer
  if (!hasPeer) {
    WebSerial.println(F("  No initial peers found by fetch_peer."));
  }
  int fetchedPeerCount = 0;
  while(hasPeer && fetchedPeerCount < MAX_FETCH_ATTEMPTS) { // Added safety break
      fetchedPeerCount++;
      // Log first few, then periodically to avoid flooding if many peers are (incorrectly) found
      if (fetchedPeerCount <= 5 || fetchedPeerCount % 10 == 0) {
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 tempPeer.peer_addr[0], tempPeer.peer_addr[1], tempPeer.peer_addr[2],
                 tempPeer.peer_addr[3], tempPeer.peer_addr[4], tempPeer.peer_addr[5]);
        WebSerial.printf("  Fetched peer %d: %s\n", fetchedPeerCount, macStr);
      }

      std::array<uint8_t, 6> mac;
      memcpy(mac.data(), tempPeer.peer_addr, 6);
      existingPeers.push_back(mac);
      hasPeer = esp_now_fetch_peer(false, &tempPeer); // Get next peer
  }
  if (fetchedPeerCount >= MAX_FETCH_ATTEMPTS && hasPeer) {
    WebSerial.printf("  Warning: Stopped fetching peers after %d attempts due to exceeding MAX_FETCH_ATTEMPTS (%d). ESP-NOW internal list might be corrupted.\n", fetchedPeerCount, MAX_FETCH_ATTEMPTS);
  }
  WebSerial.printf("  Finished fetching. Total existing peers collected into list: %d\n", existingPeers.size());

  // 2. Dann alle gesammelten Peers löschen
  WebSerial.printf("  %d vorhandene Peers gefunden. Entferne sie...\n", existingPeers.size());
  for(const auto& mac_array : existingPeers) {
      char macStr[18];
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac_array[0], mac_array[1], mac_array[2], mac_array[3], mac_array[4], mac_array[5]);
      WebSerial.printf("  Entferne Peer: %s\n", macStr);
      esp_err_t del_result = esp_now_del_peer(mac_array.data());
      if (del_result != ESP_OK && del_result != ESP_ERR_ESPNOW_NOT_FOUND) {
          WebSerial.printf("    Fehler beim Entfernen: %s\n", esp_err_to_name(del_result));
      } else {
          WebSerial.println(F("    Peer removed or not found."));
      }
  }
  WebSerial.println(F("  Finished deleting existing peers."));

  // 3. Alle gültigen Peers aus dem globalen slaveMacs Array hinzufügen
  WebSerial.println(F("  Adding peers from slaveMacs array..."));
  numRegisteredSlaves = 0; // Zähler Zuruecksetzen
  for (int i = 0; i < MAX_SLAVES; ++i) {
    // Prüfen, ob der Eintrag gültig ist (nicht alle 00 oder alle FF)
    bool isMacValid = false;
    bool isMacAllFF = true;
    bool isMacAll00 = true;
    for(int j=0; j<6; ++j) {
        if(slaveMacs[i][j] != 0xFF) isMacAllFF = false;
        if(slaveMacs[i][j] != 0x00) isMacAll00 = false;
    }
    isMacValid = !isMacAllFF && !isMacAll00;

    if (isMacValid) {
      memcpy(peerInfo.peer_addr, slaveMacs[i], 6);
      peerInfo.channel = 0; // 0 bedeutet aktueller WLAN-Kanal
      peerInfo.encrypt = false; // Keine Verschlüsselung verwenden

      esp_err_t add_result = esp_now_add_peer(&peerInfo);
      if (add_result != ESP_OK){
        // ESP_ERR_ESPNOW_EXIST ist kein echter Fehler in diesem Kontext
        if (add_result == ESP_ERR_ESPNOW_EXIST) {
           WebSerial.printf("  Peer %d bereits vorhanden: %02X:%02X:%02X:%02X:%02X:%02X\n", i + 1, slaveMacs[i][0], slaveMacs[i][1], slaveMacs[i][2], slaveMacs[i][3], slaveMacs[i][4], slaveMacs[i][5]);
           numRegisteredSlaves++; // Zählen wir ihn trotzdem
        } else {
           WebSerial.printf("  Fehler beim Hinzufügen von Peer %d (%02X:%02X:%02X:%02X:%02X:%02X): %s\n", i + 1, slaveMacs[i][0], slaveMacs[i][1], slaveMacs[i][2], slaveMacs[i][3], slaveMacs[i][4], slaveMacs[i][5], esp_err_to_name(add_result));
        }
      } else {
        WebSerial.printf("  Peer %d hinzugefügt: %02X:%02X:%02X:%02X:%02X:%02X\n", i + 1, slaveMacs[i][0], slaveMacs[i][1], slaveMacs[i][2], slaveMacs[i][3], slaveMacs[i][4], slaveMacs[i][5]);
        numRegisteredSlaves++;
      }
    }
  }
  WebSerial.printf("ESP-NOW Peer Update abgeschlossen. %d Slaves registriert. [END]\n", numRegisteredSlaves);
}

// NEU: Handler zum Abrufen des seriellen Logs
void handleGetSerialLog() {
  server.send(200, "text/plain", WebSerial.getWebLog());
}

// NEU: Handler zum Umschalten des Web-Serial-Logs
void handleToggleWebSerialLog() {
  WebSerial.enableWebLog(!WebSerial.isWebLogEnabled());
  WebSerial.print(F("Web Serial Log "));
  WebSerial.println(WebSerial.isWebLogEnabled() ? F("aktiviert.") : F("deaktiviert."));
  // Kein Redirect, da dies oft per JS/AJAX aufgerufen wird.
  // Der Status wird über /status aktualisiert, was den Button-Text ändert.
  server.send(200, "text/plain", WebSerial.isWebLogEnabled() ? "enabled" : "disabled");
}

// NEU: Handler zum Löschen des Web-Serial-Logs
void handleClearWebSerialLog() {
  WebSerial.clearWebLog();
  WebSerial.println(F("Web Serial Log geleert."));
  server.send(200, "text/plain", "cleared");
}


// Handler für die GET /update Seite (zeigt nur Anweisungen)
void handleOtaUpdatePage() {
  // Use PSTR for large static HTML content
  server.send_P(200, "text/html", PSTR(R"rawliteral(
<html>
<head>
  <title>OTA Firmware Update</title>
  <style>
    body { font-family: sans-serif; }
    .progress { width: 300px; height: 20px; border: 1px solid #ccc; margin-top: 10px; }
    .progress-bar { width: 0%; height: 100%; background-color: #4CAF50; text-align: center; line-height: 20px; color: white; }
  </style>
</head>
<body>
  <h1>OTA Firmware Update</h1>
  <p>Waehlen Sie eine .bin Datei zum Hochladen:</p>
  <form method='POST' action='/update' enctype='multipart/form-data' id='upload_form'>
    <input type='file' name='update' accept='.bin'>
    <input type='submit' value='Update Firmware'>
  </form>
  <div id='prg_wrap' style='display:none;'>
    <p>Fortschritt:</p>
    <div class='progress'>
      <div class='progress-bar' id='prg'>0%</div>
    </div>
  </div>
  <p><a href='/'>Zurueck zur Hauptseite</a></p>

  <script>
    document.getElementById('upload_form').addEventListener('submit', function(e) {
      e.preventDefault();
      var form = e.target;
      var data = new FormData(form);
      var prgWrap = document.getElementById('prg_wrap');
      var prg = document.getElementById('prg');
      prgWrap.style.display = 'block';
      prg.style.width = '0%';
      prg.textContent = '0%';
      var xhr = new XMLHttpRequest();
      xhr.open('POST', form.action, true);

      xhr.upload.addEventListener('progress', function(e) {
        if (e.lengthComputable) {
          var percentage = Math.round((e.loaded / e.total) * 100);
          prg.style.width = percentage + '%';
          prg.textContent = percentage + '%';
        }
      });
      xhr.send(data);
    });
  </script>
</body></html>)rawliteral") );
  // No need for String html object
}

// Handler für unbekannte URLs
void handleNotFound() {
  // Send in chunks to avoid large String
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(404, "text/plain", ""); // Send headers

  server.sendContent_P(PSTR("File Not Found\n\nURI: "));
  server.sendContent(server.uri());
  server.sendContent_P(PSTR("\nMethod: "));
  server.sendContent((server.method() == HTTP_GET) ? PSTR("GET") : PSTR("POST"));
  server.sendContent_P(PSTR("\nArguments: "));
  server.sendContent(String(server.args())); // String for integer to char* conversion
  server.sendContent_P(PSTR("\n"));
  for (uint8_t i = 0; i < server.args(); i++) {
    server.sendContent(" " + server.argName(i) + ": " + server.arg(i) + "\n"); // These create temp Strings
  }
  server.sendContent(""); // Finalize
}

// Handler für den Datei-Upload (wird für jeden Datenblock aufgerufen)
void handleFirmwareUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    WebSerial.printf("Update Start: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // Start update
      Update.printError(WebSerial); // Geändert zu WebSerial
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(WebSerial); // Geändert zu WebSerial
    }
     // Optional: Kurze Pause, um Watchdog zu beruhigen bei großen Dateien
     // delay(1);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) { // true to set the size to the current progress
      WebSerial.printf("Update erfolgreich: %u Bytes\n", upload.totalSize);
    } else {
      Update.printError(WebSerial); // Geändert zu WebSerial
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
      Update.end(false); // Update abbrechen
      WebSerial.println(F("Update abgebrochen"));
  }
}

// Handler, der nach erfolgreichem Upload aufgerufen wird
void handleFirmwareUploadSuccess() {
  if (Update.isFinished()) {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "Update erfolgreich! Starte neu...");
    delay(200); // Kurze Pause, damit der Browser die Antwort erhält
    ESP.restart();
  } else {
    char errBuf[60];
    snprintf(errBuf, sizeof(errBuf), "Update fehlgeschlagen! Fehlercode: %d", Update.getError());
    server.send(500, "text/plain", errBuf);
  }
}

void handleEncoderSwitchPressed(Button2& btn) {
  WebSerial.println(F("DEBUG: handleEncoderSwitchPressed aufgerufen.")); // Optional: Debug-Ausgabe anpassen
  
  if (!inMenu) { // Noch nicht im Menü
    // Immer Menü öffnen, egal ob Master oder Slave
    WebSerial.println(F("Button2: Joystick Select gedrückt ausserhalb Menue. Oeffne Hauptmenue."));
    inMenu = true;
    inSelectionMode = false; // Sicherstellen, dass wir im Hauptmenü starten
    currentMenuItem = 0; // Setze den Startpunkt auf das erste Element
    drawMenu(); // Menü sofort zeichnen
  } else if (inMenu && !inSelectionMode) { // Im Hauptmenü
      // NEU: Prüfen, ob "Zurueck" ausgewählt wurde
      if (strcmp(menuItems[currentMenuItem], PSTR("Zurueck")) == 0) {
          WebSerial.println(F("Button2: Hauptmenue: 'Zurueck' ausgewählt. Verlasse Menü."));
          inMenu = false; // Menü verlassen
          // Das Display wird in der nächsten loop()-Iteration durch updateDisplay() aktualisiert
          return; // Funktion beenden
      }
      // Prüfen, ob der ausgewählte Punkt "Web Log" ist
      else if (strcmp(menuItems[currentMenuItem], PSTR("Web Log")) == 0) { // Vergleiche mit "Web Log"
          WebSerial.println(F("Button2: Hauptmenue: Web Log ausgewaehlt. Toggle Web Log."));
          WebSerial.enableWebLog(!WebSerial.isWebLogEnabled()); // Toggle den Web Log Zustand
          drawMenu(); // Menü neu zeichnen, um den geänderten Zustand anzuzeigen
          // Bleibe im Hauptmenü, da es kein Untermenü für Web Log gibt
      } else { // Anderer Menüpunkt, der eine Auswahl hat
          WebSerial.println(F("Button2: Hauptmenue: Select gedrueckt. Wechsle zu Auswahlmodus."));
          inSelectionMode = true;
          selectionMenuItem = currentMenuItem;
          // Setze den initialen Auswahlindex auf den aktuell gespeicherten Wert
          if (selectionMenuItem == 0) currentSelectionIndex = currentShootingTimeIndex;
          else if (selectionMenuItem == 1) currentSelectionIndex = currentTargetModeIndex;
          else if (selectionMenuItem == 2) currentSelectionIndex = currentRoleIndex;
          drawMenu(); // Auswahlmodus zeichnen
      }
  } else if (inMenu && inSelectionMode) { // Im Auswahlmodus -> Auswahl bestätigen
      const char** optionsArray = nullptr;
      int numOptions = 0;
      if (selectionMenuItem == 0) { optionsArray = shootingTimeOptions; numOptions = sizeof(shootingTimeOptions) / sizeof(shootingTimeOptions[0]); }
      else if (selectionMenuItem == 1) { optionsArray = targetModes; numOptions = sizeof(targetModes) / sizeof(targetModes[0]); }
      else if (selectionMenuItem == 2) { optionsArray = roleModes; numOptions = sizeof(roleModes) / sizeof(roleModes[0]); }

      // NEU: Prüfen, ob "Zurueck" im Auswahlmodus ausgewählt wurde
      if (optionsArray != nullptr && strcmp(optionsArray[currentSelectionIndex], PSTR("Zurueck")) == 0) {
          WebSerial.println(F("Button2: Auswahlmodus: 'Zurueck' ausgewählt. Gehe zu Hauptmenü."));
          inSelectionMode = false; // Zurueck zum Hauptmenü
          drawMenu(); // Hauptmenü neu zeichnen
          return; // Funktion beenden
      }

      WebSerial.print(F("Button2: Auswahlmodus: Select gedrueckt. Bestaetige Auswahl fuer ")); WebSerial.println(menuItems[selectionMenuItem]);
      // Einstellung basierend auf selectionMenuItem und currentSelectionIndex speichern
      if (selectionMenuItem == 0) { // Schießzeit
        currentShootingTimeIndex = currentSelectionIndex;
        shootingTimeSetting = shootingTimeOptions[currentShootingTimeIndex];
        updateTimings();
      } else if (selectionMenuItem == 1) { // Zielmodus
        currentTargetModeIndex = currentSelectionIndex;
        targetMode = targetModes[currentTargetModeIndex]; // targetMode ist const char*
        if (isMaster) { // Reset nur im Master bei Moduswechsel
            abCdPhase = 0; isStartingGroupAB = true; setState(ROT, TONE_NONE);
        }
      } else if (selectionMenuItem == 2) { // Rolle
        bool oldIsMaster = isMaster;
        currentRoleIndex = currentSelectionIndex;
        currentRole = roleModes[currentRoleIndex];
        isMaster = (currentRole == roleModes[0]); // "Master"
        if (isMaster && !oldIsMaster) { // War Slave, wird Master
            WebSerial.println(F("Rolle auf Master gesetzt. ESP-NOW senden aktiv."));
            abCdPhase = 0; isStartingGroupAB = true; setState(ROT, TONE_NONE);
        } else if (!isMaster && oldIsMaster) { // War Master, wird Slave
            WebSerial.println(F("Rolle auf Slave gesetzt. ESP-NOW empfangen aktiv."));
        }
        // Rolle immer speichern, nachdem sie im Joystick-Menü geändert und bestätigt wurde
        saveRoleToEEPROM();
      }
      printCurrentSettings(); // Geänderte Einstellung ausgeben
      inSelectionMode = false; // Zurueck zum Hauptmenü
      drawMenu(); // Hauptmenü wieder zeichnen
  }
}


void handleStartStopPressed(Button2& btn) {
  WebSerial.println(F("DEBUG: handleStartStopPressed aufgerufen."));
  WebSerial.println(F("Button2: Start/Stop Taster gedrückt."));
  if (isMaster) { // Nur Master kann starten/stoppen
      if (currentState == ROT) {
        manualStartTriggered = true;
        WebSerial.println(F("Button2: Taster: Start Trigger gesetzt"));
      } else if (currentState == GELB_PREP || currentState == GRUEN || currentState == GELB_REST) {
        manualStopTriggered = true;
        WebSerial.println(F("Button2: Taster: Stop Trigger gesetzt"));
      }
  } else {
      WebSerial.println(F("Button2: Taster: Start/Stop im Slave Modus ignoriert."));
  }
}

void handleEmergencyPressed(Button2& btn) {
  WebSerial.println(F("DEBUG: handleEmergencyPressed aufgerufen."));
  WebSerial.println(F("Button2: Notknopf Taster gedrückt."));
  triggerEmergencyStop(); // Rufe die zentrale Notstopp-Funktion auf
}

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Dynamixel2Arduino.h>
#include <Vector.h>

#define WIFI_SSID "FRITZ!Box 7590 UL"        // WLAN-SSID
#define WIFI_PASSWORD "95493765313007016133" // WLAN-Passwort

// Dynamixel-Konfiguration
#define DXL_SERIAL Serial1  // Dynamixel auf Serial1
#define DEBUG_SERIAL Serial // Debug auf Serial0
#define DXL_DIR_PIN 4       // GPIO für Dynamixel-Richtung
#define DXL_BROADCAST_ID 254

// Dynamixel Commands
#define DXL_

const unsigned long DXL_BAUDRATE = 1000000;
const float DXL_PROTOCOL = 1.0;

int servo_angle = 0; //

IPAddress staticIP(192, 168, 0, 199); // Statische IP-Adresse
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 0, 1);
IPAddress secondDNS(0, 0, 0, 0);

// UDP-Konfiguration
WiFiUDP udp;
const unsigned int localUdpPort = 12345; // Port des ESP32
char incomingPacket[512];                // Puffer für eingehende UDP-Pakete

int store_elements[DXL_BROADCAST_ID - 1];
typedef Vector<int> TDynaList;

TDynaList dynaList(store_elements);
char buffer[200];

// Dynamixel-Objekt
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void scanDynamixels(TDynaList &list, uint8_t opMode = OP_POSITION);
void handleIncomingMessage(String msg);
void publishSensorData();

void setup()
{
  // Debug starten
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
    ;

  // Dynamixel starten
  DXL_SERIAL.begin(DXL_BAUDRATE, SERIAL_8N1, 16, 17);
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  delay(1000);
  DEBUG_SERIAL.println("Dynamixel gestartet!");

  // WLAN verbinden
  WiFi.config(staticIP, gateway, subnet, primaryDNS, secondDNS); // Statische IP einstellen
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    DEBUG_SERIAL.print(".");
  }
  DEBUG_SERIAL.println("\nWLAN verbunden!");
  DEBUG_SERIAL.println(WiFi.localIP());

  // UDP starten
  udp.begin(localUdpPort);
  DEBUG_SERIAL.printf("UDP Server gestartet auf Port %d\n", localUdpPort);

  // Dynamixel-Scan
  scanDynamixels(dynaList, OP_POSITION);
}

void loop()
{
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    // Daten empfangen
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0)
    {
      incomingPacket[len] = 0; // Null-terminieren
    }
    DEBUG_SERIAL.printf("Empfangen: %s\n", incomingPacket);

    // Nachricht verarbeiten
    handleIncomingMessage(String(incomingPacket));
  }

  // Sensordaten senden (optional, falls du regelmäßig Daten senden willst)
  publishSensorData();
}

/**
 * @brief Scan für Dynamixels. Wenn gefunden, werden die IDs in eine Liste aufgenommen.
 */
void scanDynamixels(TDynaList &list, uint8_t opMode)
{
  if (dynaList.size() < DXL_BROADCAST_ID)
  {
    for (uint8_t id = 0; id < DXL_BROADCAST_ID; id++)
    {
      if (dxl.ping(id))
      {
        DEBUG_SERIAL.printf("ID (%3d), Model: %3d\n", id, dxl.getModelNumber(id));
        dynaList.push_back(id);
        dxl.torqueOff(id);
        dxl.setOperatingMode(id, opMode);
        dxl.torqueOn(id);
      }
    }
    DEBUG_SERIAL.printf("Number of Dynamixels found: {%3d}\n", dynaList.size());
  }
}

/**
 * @brief Eingehende UDP-Nachricht verarbeiten
 * IncommingMessage ist ein Steuerbefehl für die Motoren
 *
 */
void handleIncomingMessage(String msg)
{
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, msg);

  if (error)
  {
    DEBUG_SERIAL.println("JSON-Parsing fehlgeschlagen!");
    return;
  }
  DEBUG_SERIAL.println("Parsing done");

  //  const char *command = doc["command"];
  const char *command = doc["cmd"];
  if (strcmp(command, "move_motor") == 0)
  {
    DEBUG_SERIAL.println("set DEGREES");
    int motor_id = doc["id"];
    float position = doc["position"];
    int unit = UNIT_DEGREE;
    dxl.setGoalPosition(motor_id, position, UNIT_DEGREE); // Position setzen
    DEBUG_SERIAL.printf("Motor %d auf Winkel %f° bewegt\n", motor_id, position);
  }
}

/**
 * @brief Sensordaten an den letzten Sender senden
 */
void publishSensorData()
{
  DynamicJsonDocument doc(1024);
  doc["sensor"] = random(0, 1024);                 // Beispiel: Zufallswert
  doc["motor_status"] = dxl.getPresentPosition(1); // Beispiel: Motorposition für Motor 1
  String jsonStr;
  serializeJson(doc, jsonStr);

  // Sensordaten an den Sender senden
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print(jsonStr);
  udp.endPacket();
}

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Dynamixel2Arduino.h>
#include "rover.h"
#include "rover_utils.h"
#include <Vector.h>

#include <Arduino.h>

const unsigned long DXL_BAUDRATE = 1000000;
const float DXL_PROTOCOL = 1.0;

int store_elements[DXL_BROADCAST_ID - 1];
typedef Vector<int> TDynaList;
TDynaList dynaList(store_elements);
char buffer[200];

// Dynamixel-Objekt
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// void scanDynamixels(TDynaList &list, uint8_t opMode = OP_POSITION);
bool scanDynamixels();
void goStartPosition();
void setWheelSpeed(Dynamixel2Arduino *dxl, uint8_t id, uint16_t speed);
void testSetRoverVelocity(Dynamixel2Arduino *dxl);

int8_t dir = 1;
int speed = -1023;
int step = 50;
int count = 1;

#define MAX_SPEED ;

void setup()
{
  // Debug starten
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
    ;

  delay(500); // warten um VSCode zeit zu geben den Monitor zu öffnen

  // Dynamixel starten
  DXL_SERIAL.begin(DXL_BAUDRATE, SERIAL_8N1, 16, 17);
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  delay(2000);

  // Dynamixel-Scan
  // alle gefundenen Servos erst einmal auf MODE: OP_POSTION setzen (Winkel 0-300Grad)
  // scanDynamixels(dynaList, OP_POSITION);
  if (!scanAllDynamixels(&dxl))
  {
    // DEBUG_SERIAL.println("SERVO-Liste falsch konfiguriert. Angegebene Servos nicht gefunden.");
    while (true)
    {
      delay(1000);
      DEBUG_SERIAL.print("ERROR ");
    }
  }

  goStartPosition();
  DEBUG_SERIAL.println("Steering in Startposition");

  for (int id = 0; id < 4; id++)
  {
    setWheelMode(&dxl, dxl_ids_velocity[id][0]);
    DEBUG_SERIAL.printf("(%ID) CW(%d) CCW(%d)\n",
                        id,
                        dxl.readControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id),
                        dxl.readControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id));
  }
  DEBUG_SERIAL.println("Antriebseinheiten im Wheel-Mode");
}

void loop()
{

  // delay innerhalb der test-Routine
  testSetRoverVelocity(&dxl);
}

void goStartPosition()
{
}

/**
 * @brief Scannt die Dynamixel-Kette und sucht ob die definierten IDs auch verfügbar sind.
 * Wenn nein liefert die Funktion false zurück. Wurde ein Serve gefunden, wird auch direkt der
 * Mode für diesen Servo gesetzt. OP_VELOCITY (für Antriebsservos), OP_POSITION (für lenkservos)
 *
 */
bool scanDynamixels()
{
  size_t s = ARRAY_SIZE(dxl_ids_steering);
  bool found = true;
  for (uint8_t id = 0; id < s; id++)
  {
    // Steering Servos
    if (dxl_ids_steering[id] > 0 && dxl.ping(dxl_ids_steering[id]))
    {
      DEBUG_SERIAL.printf("STEERING-Servo ID:(%2d) MODE: (%d) MODEL:(%3d)\n",
                          dxl_ids_steering[id], OP_POSITION, dxl.getModelNumber(id));
      dxl.torqueOff(id); // Torque abschalten um den Mode zu setzen
      dxl.setOperatingMode(id, OP_POSITION);
      dxl.torqueOn(id); // Troque wieder aktivieren um den Servo nutzen zu können
    }
    else
    {
      if (dxl_ids_steering[id] == 0)
        continue;
      DEBUG_SERIAL.printf("STEERING-Servo ID:(%2d) Servo not found - error - check dxl_ids_steering\n",
                          dxl_ids_steering[id]);
      found = false;
    }
    // Antriebsservos
    if (dxl_ids_velocity[id][0] > 0 && dxl.ping(dxl_ids_velocity[id][0]))
    {
      DEBUG_SERIAL.printf("VELOCITY-Servo ID:(%2d) MODE: (%d) MODEL:(%3d)\n",
                          dxl_ids_velocity[id][0], OP_VELOCITY, dxl.getModelNumber(id));
      dxl.torqueOff(id); // Torque abschalten um den Mode zu setzen
      dxl.setOperatingMode(id, OP_VELOCITY);
      dxl.torqueOn(id); // Troque wieder aktivieren um den Servo nutzen zu können
    }
    else
    {
      if (dxl_ids_velocity[id][0] == 0)
        continue;
      DEBUG_SERIAL.printf("VELOCITY-Servo ID:(%2d) Servo not found - error - check dxl_ids_velocity\n",
                          dxl_ids_velocity[id]);
      found = false;
    }
  }
  return found;
}

/**
 * @brief Testest die Funktion setRoverVelocity mit diversen Parametereinstellungen
 */
void testSetRoverVelocity(Dynamixel2Arduino *dxl)
{
  uint16_t test_speeds[] = {100, 200, 400, 600, 800, 1023, 0};         // Verschiedene Geschwindigkeitsstufen
  float test_gains[] = {0.0, 0.25, 0.5, 0.65, 1.0, -0.25, 0.5, 0.75, -1.0}; // Verstärkungs- und Dämpfungswerte
  uint8_t test_dirs[] = {1, 0};                                        // Richtung: Vorwärts und Rückwärts

  for (float gain : test_gains)
  {
    DEBUG_SERIAL.printf("(1) NEW GAIN => %2.1f \n", gain);
    for (uint16_t speed : test_speeds)
    {
      DEBUG_SERIAL.printf("--(2) NEW SPEED => %4d \n", speed);

      for (uint8_t dir : test_dirs)
      {
        DEBUG_SERIAL.printf("----(3) NEW dir SPEED(%4d) GAIN (%2.1f) DIR (%1d) \n", speed, gain, dir);
        setRoverVelocity(dxl, speed, gain, dir);
        delay(1000); // Kurze Verzögerung, um die Effekte zu beobachten
      }
    }
  }
}
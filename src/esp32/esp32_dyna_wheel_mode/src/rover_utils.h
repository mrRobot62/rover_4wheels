#ifndef _ROVER_UTILS_H_
#define _ROVER_UTILS_H_

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "rover.h"

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// Funktion zur Überprüfung, ob eine ID im Array vorhanden ist
/**
 * @brief prüft ob ein Wert im übergebenen Array enthalten ist
 * Wird z.B. benötigt um herauszufinden ob eine Servo-ID in der
 * Liste der benutzten Servos steht
 *
 * @param id DynamixelID
 * @param *array Zeiger auf das zu prüfende Array
 * @param size Größe des zu prüfenden Arrays
 */
uint8_t id_exists(uint8_t id, const uint8_t *array, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        if (array[i] == id)
        {
            return 1; // ID gefunden
        }
    }
    return 0; // ID nicht gefunden
}

/**
 * @brief Konvertiert einen Geschwindigkeitswert von 0..100 in einen Geschwindigkeitswert für Dynamixel. CCW 1..1023, CW 1024..2047, 0=stop
 * (aktuell nur UNIT_RAW)
 */
void setSpeed(Dynamixel2Arduino *dxl, uint8_t id, int value, int min = -100, int max = 100, uint8_t unit = UNIT_RAW)
{
    int velocity;
    if (value > max)
        value = max;
    if (value < -min)
        value = -min;

    if (value == 0)
    {
        velocity = 0;
    }
    else if (value > 0)
    {
        velocity = map(value, 1, max, 1024, 2047);
    }
    else
    {
        velocity = map(value, -min, -1, 1023, 1);
    }
    dxl->setGoalVelocity(id, velocity, UNIT_RAW);
}

/**
 * @brief Konvertiert einen Winkelwert in einen Positionswert zw. 0..1023 und setzt die Position des Servos
 * Der übergebene Wert kann ein Prozentwert zwischen 0.0..100.0, ein Winkel 0.0..300.0 oder ein Rohwert von 0..1023 sein
 * Unterschieden wird dies über den Paramter unit. Die Werte für min/max müssen an den übergabe Wert (value) entsprechend angepasst werden beim
 * Aufruf der funktion
 *
 * @param dxl Zeiger auf das dxl-objekt
 * @param id Dynamixel ID
 * @param min/max Min/Max bezogen auf den Value
 * @param float wenn UNIT_DEGREE dann Wert zwischen 0..300,
 *          wenn UNIT_RAW, dann Wert zwischen -1.0 .. 0 .. 1.0,
 *          wenn UNIT_PERCENT, dann value zwischen -100.0 .. 0 .. 100.0,
 *          default = UNIT_RAW
 * @param velocity kann genutzt werden um die Beschleunigung anzupassen. Default 0 (max) 1=sehr langsam CW, 1023 max CW, 1024 langsam CCW, 2047 max CCW
 */
void setAngle(Dynamixel2Arduino *dxl, uint8_t id, float value, float min = -1.0, float max = 1.0, uint8_t unit = UNIT_RAW, uint16_t velocity = 0)
{
    float angle;
    long v;
    if (value > max)
        value = max;
    if (value < min)
        value = min;
    switch (unit)
    {
    case UNIT_DEGREE:
        angle = value;
        break;
    case UNIT_PERCENT:
        v = long(value * 10.0);
        angle = map(v, long(min), long(max), 0, 1023);

        break;
    default: // UNIT_RAW
        unit = UNIT_RAW;
        // um die Nachkommastellen aufgelöst zu bekommen, mit 100 multiplizieren
        v = long(value * 100.0);
        angle = map(v, long(min * 100.0), long(max * 100.0), 0, 1023);
        break;
    }
    // angle = map(value, min,max, 1, 1023);
    dxl->writeControlTableItem(MOVING_SPEED, id, 10);
    dxl->setGoalVelocity(id, velocity, UNIT_RAW);
    dxl->setGoalPosition(id, angle, unit);
}

/**
 * @brief gibt den aktuellen Winkel der Servostellung zurück für Servo id zurück
 */
float getPostionInDegree(Dynamixel2Arduino *dxl, uint8_t id)
{
    float raw = dxl->getPresentPosition(id, UNIT_RAW);
    return map(raw, 0, 1023, 0, 300);
}

void printCurrentAngles(Dynamixel2Arduino *dxl)
{
    size_t s = ARRAY_SIZE(dxl_ids_steering);
    float angle;
    for (uint8_t id = 0; id < s; id++)
    {
        if (dxl_ids_steering[id] == 0)
            continue;
        angle = getPostionInDegree(dxl, dxl_ids_steering[id]);
        DEBUG_SERIAL.printf("SERVO(%02d) - Winkel: (%3.2f)° \n", dxl_ids_steering[id], angle);
    }
}

/**
 * @brief setzt eine physikalisches Winkel-Limit für den Servo. Wird im EEPROM gespeichert und ist auch nach einem Neustart vorhanden !
 * Reset kann nur mit min=0, max=300 auf den maximal Winkelausschlag zurück gesetzt werden !
 */
void saveAngleLimits(Dynamixel2Arduino *dxl, uint8_t id, uint16_t min = 0, uint16_t max = 300)
{
    dxl->torqueOff(id);
    min = map(min, 0, 300, 0, 1023);
    max = map(max, 0, 300, 0, 1023);
    dxl->writeControlTableItem(6, id, min);
    dxl->writeControlTableItem(8, id, max);
    dxl->torqueOn(id);
}

void resetAllServos(Dynamixel2Arduino *dxl)
{
    DEBUG_SERIAL.println("*********************** FACTORY RESET ALL SERVOS *************************");
    dxl->factoryReset(0xFE, 0x01);
    delay(1000);
}

/**
 * @brief setzt für den Servo den Wheel Modus. Wird im EEPROM des Servos gespeichert
 */
void setWheelMode(Dynamixel2Arduino *dxl, uint8_t id)
{
    dxl->writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0);
    dxl->writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
}

/**
 * @brief führt ein PING über die komplette Chain durch
 */
bool scanAllDynamixels(Dynamixel2Arduino *dxl, uint8_t from = 1, uint8_t to = 253)
{
    bool found = false;
    for (int id = from; id < to; id++)
    {
        if (dxl->ping(id))
        {
            DEBUG_SERIAL.printf("Serve (%3d) found\n", id);
            found = true;
        }
    }
    if (!found)
    {
        DEBUG_SERIAL.print("------ NO SERVOS FOUND ------");
    }
    return found;
}

/**
 * @brief Geschwindigkeit des Rovers für alle Antriebsservos setzen.
 * Die Geschwindigkeit wird in einem Bereich von 0-1023 angegeben. Forward/Backward wird über DIR angeben (0=Backward, 1=Forward(default))
 * Verstärkungs- oder Dämpfungswert über GAIN. Wertebereich von -1.0 (Dämpfen) bis +1.0 (Verstärken)
 * Nutzbar um den eigentlichen Geschwindigkeitswert beibehalten und trotzdem die Geschwindigkeit (z.B. in Kurven) anzupassen
 *
 * @param dxl Pointer auf das Dynamixel2Arduino Objekt
 * @param speed Geschwindigkeit zwischen 0-1023
 * @param gain Verstärkung-/Dämpfungsfaktor von -1.0(dämpfen) bis +1.0(Verstärken)
 * @param dir Forward(1), Backward(0)
 *
 */
void setRoverVelocity(Dynamixel2Arduino *dxl, uint16_t speed, float gain = 0.0, uint8_t dir = 1)
{
    if (speed > 1023)
        speed = 1023; // Begrenzung des Speed-Werts
    if (gain < -1.0)
        gain = -1.0; // Begrenzung von gain
    if (gain > 1.0)
        gain = 1.0;

    for (int i = 0; i < ARRAY_SIZE(dxl_ids_velocity); i++)
    {
        uint8_t servo_id = dxl_ids_velocity[i][0];
        uint16_t base_speed = dxl_ids_velocity[i][1];
        uint16_t adjusted_speed;

        // Berechnung des angepassten Geschwindigkeitswerts
        int32_t new_speed = speed + (speed * gain);

        if (gain > 0.0)
        {
            // Verstärkung: Obergrenze prüfen
            if (base_speed == 0)
            { // CCW
                adjusted_speed = (new_speed > 1023) ? 1023 : (base_speed + new_speed);
            }
            else
            { // CW
                adjusted_speed = (new_speed > 2047) ? 2047 : (base_speed + new_speed);
            }
        }
        else if (gain < 0.0)
        {
            // Dämpfung: Untergrenze prüfen
            if (base_speed == 0)
            { // CCW
                adjusted_speed = (new_speed < 0) ? 0 : (base_speed + new_speed);
            }
            else
            { // CW
                adjusted_speed = (new_speed < 1024) ? 1024 : (base_speed + new_speed);
            }
        }
        else
        {
            adjusted_speed = (base_speed + speed);
        }

        // Richtung umkehren, wenn dir == 0 (Rückwärtsmodus)
        if (dir == 0)
        {
            adjusted_speed = (adjusted_speed < 1024) ? adjusted_speed + 1024 : adjusted_speed - 1024;
        }

        // Geschwindigkeitswert an den Servo senden
        DEBUG_SERIAL.printf("\t|(%d) SPEED(%4d)", servo_id, adjusted_speed);
        dxl->setGoalVelocity(servo_id, adjusted_speed, UNIT_RAW);
    }
    DEBUG_SERIAL.println();
}

#endif
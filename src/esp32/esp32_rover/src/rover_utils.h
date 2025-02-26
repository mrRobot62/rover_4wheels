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
uint8_t id_exists(uint8_t id, const uint8_t *array, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        if (array[i] == id) {
            return 1; // ID gefunden
        }
    }
    return 0; // ID nicht gefunden
}


/**
 * @brief Konvertiert einen Geschwindigkeitswert von 0..100 in einen Geschwindigkeitswert für Dynamixel. CCW 1..1023, CW 1024..2047, 0=stop
 * (aktuell nur UNIT_RAW)
 */
void setSpeed(Dynamixel2Arduino *dxl, uint8_t id, int value, int min=-100, int max=100, uint8_t unit=UNIT_RAW) {
    int velocity;
    if (value > max) value = max;
    if (value < -min) value = -min;

    if (value == 0) {
        velocity = 0;
    } else if (value > 0) {
        velocity = map(value, 1,max,1024,2047);
    } else {
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
void setAngle(Dynamixel2Arduino *dxl, uint8_t id, float value, float min=-1.0, float max=1.0, uint8_t unit = UNIT_RAW, uint16_t velocity=0) {
    float angle;
    long v;
    if (value > max) value = max;
    if (value < min) value = min;
    switch (unit) {
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
            angle = map(v,long(min*100.0), long(max*100.0), 0, 1023);
            break;

        }
    //angle = map(value, min,max, 1, 1023);
    dxl->writeControlTableItem(MOVING_SPEED, id, 10);
    dxl->setGoalVelocity(id, velocity, UNIT_RAW);
    dxl->setGoalPosition(id, angle, unit);
}

/**
 * @brief gibt den aktuellen Winkel der Servostellung zurück für Servo id zurück
 */
float getPostionInDegree(Dynamixel2Arduino *dxl, uint8_t id) {
    float raw = dxl->getPresentPosition(id, UNIT_RAW);
    return map(raw, 0,1023, 0, 300);
}

void printCurrentAngles(Dynamixel2Arduino *dxl) {
    size_t s = ARRAY_SIZE(dxl_ids_steering);
    float angle;
    for (uint8_t id = 0; id < s; id++) {
        if (dxl_ids_steering[id] == 0) continue;
        angle = getPostionInDegree(dxl, dxl_ids_steering[id]);
        DEBUG_SERIAL.printf("SERVO(%02d) - Winkel: (%3.2f)° \n", dxl_ids_steering[id], angle);
    }
}

/**
 * @brief setzt eine physikalisches Winkel-Limit für den Servo. Wird im EEPROM gespeichert und ist auch nach einem Neustart vorhanden !
 * Reset kann nur mit min=0, max=300 auf den maximal Winkelausschlag zurück gesetzt werden !
 */
void saveAngleLimits(Dynamixel2Arduino *dxl, uint8_t id, uint16_t min=0, uint16_t max=300) {
    dxl->torqueOff(id);
    min = map (min, 0, 300, 0, 1023);
    max = map (max, 0, 300, 0, 1023);
    dxl->writeControlTableItem(6, id, min);
    dxl->writeControlTableItem(8, id, max);
    dxl->torqueOn(id);
}

void resetAllServos(Dynamixel2Arduino *dxl) {
    DEBUG_SERIAL.println("*********************** FACTORY RESET ALL SERVOS *************************");
    dxl->factoryReset(0xFE, 0x01);
    delay(1000);
}
#endif
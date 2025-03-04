
#ifndef _ROVER_H_
#define _ROVER_H_
#include <Arduino.h>

/**
 * verfügbare SSID zur Kopplung definieren
 */
#define WIFI_SSID "FRITZ!Box 7590 UL"        // WLAN-SSID
#define WIFI_PASSWORD "95493765313007016133" // WLAN-Passwort

// Dynamixel-Konfiguration
#define DXL_SERIAL Serial1  // Dynamixel auf Serial1
#define DEBUG_SERIAL Serial // Debug auf Serial0
#define DXL_DIR_PIN 4       // GPIO für Dynamixel-Richtung
#define DXL_BROADCAST_ID 254

#define MOVING_SPEED 32 // das ist die Adresse für Beschleunigung in der DynamixelControlTable

/*
 * Wo ist der Servo verbaut (RoverPosition)
 */
enum ServoPosition
{
    VL = 0,
    VR = 1,
    HL = 2,
    HR = 3
};

/**
 * Lenkservo oder Antriebsservo
 */
enum ServoType
{
    STEERING = 0,
    WHEEL = 1
};

/**
 * Ein RoverServo
 */
typedef struct
{
    uint8_t id;
    ServoPosition servo_position;
    ServoType servo_type;

    // nur bei ServoType STEERING wichtig
    int START_ANGLE_POSITION = 0;
    int MID_ANGLE_POSITION = 511;
    int CW_ANGLE_LIMIT = 0;
    int CCW_ANGLE_LIMIT = 1023;

    // nur by ServoType WHEEL wichtig
    int BACKWARD_START_VELOCITY = 1024;
    int FORWARD_START_VOLOCITY = 0;
} RoverServo;

uint16_t dxl_ids_steering[4][5] = {
    // id, min, max, start, mid
    {8, 275, 850, 315, 511},
    {7, 275, 850, 511},
    {17, 275, 850, 511},
    {18, 275, 850, 511},

}; // VL, VR, HL, HR
uint16_t dxl_ids_velocity[4][2] = {
    {4, 0},     // VL=CCW, start von 0-1023
    {11, 1024}, // VR=CW, start von 1024-2047
    {3, 0},     // HL=CCW
    {1, 1024}   // HR=CW

}; // VL, VR, HL, HR

#endif
#include <Dynamixel2Arduino.h>
#include "DynamicArray.h"

//---------------------------------------------------------------------
// 2025-02-24
//
// Dyna-Angle-Wizard
//
// Alle Servos sind ohne Torque und können bewegt werden.
// Wärend der Bewegung wird im Terminal die aktuelle Grad und der dazugehörige
// RAW-Wert angezeigt.
// 
// jeder Steering-Servo (VL, VR, HL, HR) muss auf mehrere Position gebracht werden
// Einstellungen merken und später in der firmware des Rovers speichern
//
//
//---------------------------------------------------------------------

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))


#define DEBUG_SERIAL Serial
#define DXL_SERIAL Serial2

#if defined(CONFIG_IDF_TARGET_ESP32)
#define TX_PIN 17 // GPIO für UART TX
#define RX_PIN 16 // GPIO für UART RX
#define DIR_PIN 4 // GPIO zur Steuerung des OE-Pins (HC125)
#define BOARD_TYPE "ESP32-WROOM"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define TX_PIN 17 // GPIO für UART TX
#define RX_PIN 16 // GPIO für UART RX
#define DIR_PIN 4 // GPIO zur Steuerung des OE-Pins (HC125)
#define BOARD_TYPE "ESP32-S3"
#endif

#define BAUDRATE 1000000 // Dynamixel-Standardbaudrate
#define DXL_ID_P 14         // Lenkung
#define DXL_ID_V 7          // Geschwindigkeit

#define MAX_BAUD 1
// const int32_t baud[MAX_BAUD] = {57600, 115200, 1000000, 2000000, 3000000};
const int32_t baud[MAX_BAUD] = {1000000};

Dynamixel2Arduino dxl(DXL_SERIAL, DIR_PIN);

uint8_t ignore_dxl_ids[] = {4, 11, 1, 3};
char * steps[] = {
    "Step1 - Servomotoren mittig stellen - Winkelangabe sollte 150° und raw = 512 sein, \nwenn nicht => Antriebseinheit demontieren und justieren\nSPACE drücken zum messen\n\n",
    "Step2 - Servo CCW bis min drehen - Werte merken\nSPACE drücken zum messen\n\n",
    "Step3 - Servo CW bis max drehen - Werte merken\nSPACE drücken zum messen\n\n"
};

#define MAX_STEPS 3
int step=0;

int i = 1;
long last_millis;
int last_dir_state = HIGH;

#define MAX_SERVOS 252

bool running = true;

typedef struct {
    int id;
    int model;
    float currentDegree;
    int currentRaw;

} DynamixelServo;

DynamixelServo servo;
DynamicArray<DynamixelServo> servos;

uint8_t dxl_count = 0;

// Funktion zur Überprüfung, ob eine ID im Array vorhanden ist
uint8_t id_exists(uint8_t id, const uint8_t *array, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        if (array[i] == id) {
            return 1; // ID gefunden
        }
    }
    return 0; // ID nicht gefunden
}

/**
 * @brief führt einen Scan durch und zeigt alle verfügbaren Dynamixel-Servis mit ihrer ID an
 */
int scan() {
    uint8_t index;
    bool found = false;
    DEBUG_SERIAL.println("search .....");
    for (int8_t protocol = 1; protocol < 2; protocol++)
    {
        // Set Port Protocol version. This has to match with DYNAMIXEL protocol version.
        dxl.setPortProtocolVersion((float)protocol);
        DEBUG_SERIAL.printf("SCAN PROTOCOL {%0:1.1f}, RX_PIN: {%1:02d}, TX_PIN: {%2:02d}, DIR_PIN: {%3:02d}\n", protocol, RX_PIN, TX_PIN, DIR_PIN);
        // DEBUG_SERIAL.println(protocol);

        for (index = 0; index < MAX_BAUD; index++)
        {
            // Set Port baudrate.
            DEBUG_SERIAL.print("SCAN BAUDRATE ");
            DEBUG_SERIAL.println(baud[index]);
            dxl.begin(baud[index]);

            for (int id = 0; id < DXL_BROADCAST_ID; id++)
            {
                // iterate until all ID in each buadrate is scanned.
                if (dxl.ping(id))
                {
                    DEBUG_SERIAL.print("ID : ");
                    DEBUG_SERIAL.print(id);
                    DEBUG_SERIAL.print(", Model Number: ");
                    DEBUG_SERIAL.print(dxl.getModelNumber(id));
                    DEBUG_SERIAL.println();
                    found = true;
                    dxl.torqueOff(id);
                    if (id_exists(id, ignore_dxl_ids, ARRAY_SIZE(ignore_dxl_ids))) {
                        DEBUG_SERIAL.printf("Ignore servoe (%d) - continue");
                        continue;
                    }
                    servo.id = id;
                    servo.model = dxl.getModelNumber(id);
                    servo.currentDegree = dxl.getPresentPosition(id, UNIT_DEGREE);
                    servo.currentRaw = dxl.getPresentPosition(id, UNIT_RAW);
                    servos.push(servo);
                    dxl_count++;
                }
            }
        }
    }
    DEBUG_SERIAL.printf("search finished. (%d) found............\n", dxl_count);
    return found;
}

void tableHeader() {
    DEBUG_SERIAL.println("| STEP | ID | MODEL | DEGREE  | RAW  |");
    DEBUG_SERIAL.println("|------|----|-------|---------|------|");
}

//void tableRow(uint8_t id, uint16_t model, float degree, uint16_t raw) 
void tableRow(DynamixelServo servo) {
        DEBUG_SERIAL.printf("|   %1d  | %03d | %4d | %6.2f° | %4d | \n", 
            step,
            servo.id,
            servo.model,
            servo.currentDegree,
            servo.currentRaw
        );
}

void tableFooter() {
    DEBUG_SERIAL.println("|====|=======|=========|======|");
    DEBUG_SERIAL.println("q = quit\tSPACE = nächster Step\tw|n=Step wiederholen\n\n");
}

void scanDynamixelID(DynamixelServo servo, Dynamixel2Arduino *dxl) {
    servo.currentDegree = dxl->getPresentPosition(servo.id, UNIT_DEGREE);
    servo.currentRaw = dxl->getPresentPosition(servo.id, UNIT_RAW);
    tableRow(servo);
}

void waitForSpace() {
    while (true) {
        if (Serial.available()) {
            char input = Serial.read();
            if (input = ' ') {
                break;
            }
        }
    }
    if (++step >= MAX_STEPS) {
        step=0;
    }
}

void setup()
{
    DEBUG_SERIAL.begin(115200);
    while (!DEBUG_SERIAL)
        ; // DEBUG_SERIAL until the serial port is opened

    // DIR_PIN konfigurieren
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, HIGH); // Standard: Empfangsmodus (Buffer hochohmig)

    // DXL_SERIAL.begin(57600, SERIAL_8N1, RX_PIN, TX_PIN);
    //  Dynamixel-Bibliothek starten
    delay(2000);


    if (!scan()) {
        DEBUG_SERIAL.println("No Dynamixels found - restart/reset ESP");
        while(true) {;}
    }

    DynamixelServo *raw_list = servos.to_array();
}

void loop() {
    if (Serial.available()) {
        char input = Serial.read();
        if (input == 'q' || input == 'Q' ) {
            tableFooter();
            DEBUG_SERIAL.println("RESET esp32 for restart");
            while (true);
        } else if (input = 'w' || input == 'n') {
            step--;
            if (step < 0) step=0;
            if (step >= MAX_STEPS) step = 0;
            DEBUG_SERIAL.println("STEP wiederholen");
        }
    }

    if (running) {
        DEBUG_SERIAL.println(steps[step]);
        waitForSpace();
        tableHeader();
        servos.iterate(scanDynamixelID, &dxl); 
        tableFooter();
    }
}
 



// if (!dynas_available) {
//     if (millis() - last_millis > 3000) {
//         DEBUG_SERIAL.println("No Dynamixel servos found");
//         last_millis = millis();
//     }
// }
// else {
//     for (uint8_t i=0; i < ARRAY_SIZE(dxl_ids_velocity); i++) {
//         bool printing = false;
//         if (dxl_ids_velocity[i] > 0) {
//             currentVelocity[i] = dxl.getPresentVelocity(dxl_ids_velocity[i]);
//             DEBUG_SERIAL.printf("| VELOCITY -> ID(%02d) - Velocity (%04d)", dxl_ids_velocity[i], currentVelocity[i]);
//             printing = true;
//         }
//         if (dxl_ids_steering[i] > 0) {
//             currentSteering[i] = dxl.getPresentPosition(dxl_ids_steering[i]);
//             DEBUG_SERIAL.printf("| STEERING -> ID(%02d) - Position (%04d)", dxl_ids_steering[i], currentSteering[i]);
//             printing = true;
//         }
//         if (printing) DEBUG_SERIAL.printf(" ---- Speed (%03d) Angle(%03.2f) \n", speed, angle);
//     }

//     if (millis() - last_millis > 1000) {
//         if (dir) {
//             speed += SPEED_OFFSET;
//             if (speed >= 100 ) dir = false;
//         }
//         else {
//             speed -= SPEED_OFFSET;
//             angle - ANGLE_OFFSET;
//             if (speed <= -100 ) dir = true;
//         }

//         if (dira) {
//             angle += ANGLE_OFFSET;
//             if (angle >= angle_range[1]) {
//                 dira = false;
//             }
//         }
//         else {
//             angle -= ANGLE_OFFSET;
//             if (angle <= angle_range[0]) {
//                 dira = true;
//             }

//         }

//         for (uint8_t i=0; i < ARRAY_SIZE(dxl_ids_velocity);i++) {
//             if (dxl_ids_velocity[i] > 0) {
//                 setSpeed(dxl_ids_velocity[i], speed);
//             }
//             if (dxl_ids_steering[i] > 0) {
//                 setAngle(dxl_ids_steering[i], angle, angle_range[0], angle_range[1]);
//             }
//         }
//         last_millis = millis();
//     }
//     delay(250);
// }
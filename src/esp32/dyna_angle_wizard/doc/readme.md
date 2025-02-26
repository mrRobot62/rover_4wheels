# Dynamixel Angle Wizard
Dieses kleine Terminal Programm läuft ausschließlich nur auf dem ESP32 und muss dort explizit geflasht werden.

Die Idee hier ist, das für den Rover vier Antriebseinheiten verwendet werden. Jede Antriebseinheit besteht aus einem Wheel-Motor (Antrieb) und einem Steering-Motor (Lenkung).

Jede Antriebseinheit hat ihre eigenen Winkelbegrenzungen in CW und CCW. Um sicherzu stellen das der jeweilige Servo nicht in eine Position gedreht wird die einen Schaden verursachen kann, müssen diese Winkelbegrenzungen zur Compile-zeit der Rover-Firware bekannt sein.

Dieses tool fragt über das Terminal bestimmte Winkelstellungen ab (einzeln für alle 4 Antriebseinheiten). Die ermittelten Werte müssen anschließend in der Firmware im zwei dimensionalen Array `xxxxxx` eingetragen werden.

## Flashen
Hier gibt es keine Besonderheiten. Programm auf den ESP32 flashen und im Terminal (Serial-Monitor) die Ausgaben betrachten ()

## Einstellung
1. ESP32 mit den Dynamixels verbinden
2. Stromversorgung (11.1-12.0V) anschließen
3. ESP32 mit USB verbinden und Terminal öffen
4. ggf. nochmals den RESET-Button klicken
5. Anzeige im Terminal

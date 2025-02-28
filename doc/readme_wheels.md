# 4-Wheel Ansteuerung
Alle vier Räder können getrennt angesteuert werden. Eine Wheeleinheit besteht aus je zwei Dynamixel AX12-Servos. Ein Servo der das Rad antreibt und dementsprechend mit `OP_VELOCITY` angesteuert werden und ein Servo für die Lenkung der Radeinheit zuständig ist. Diese Servos werden über `OP_POSITION` angesteuert. Im weiteren wird eine Kombination als Antriebseinheit bezeichnet

# Geometrie
Vier Antriebseinheiten **VL, VR, HL, HR**. 
Interne Numerierung : **VL=0, VR=1, HL=2, HR=3**

## Technische Abstände und Winkel
| Technische Daten                       | Wert | Info                              |
| -------------------------------------- | :--: | --------------------------------- |
| Spurweite VL/VR                        | ?mm  | Spurweite vorne und hinten gleich |
| Spurweite HR/HL                        | ?mm  | Spurweite vorne und hinten gleich |
| Radstand V + H                         | ?mm  |                                   |
| Lenkwinkel                             |      | 180°                              | 
| Raddurchmesser mm	| 60mm  |                                   |

## Grundeinstellung des Rovers - Lenkgeometrie
| Technische Daten                       | Wert | Info                              |
| -------------------------------------- | :--: | --------------------------------- |
| Vorne links                            | 45°  | Vorwärts/Rückwärts                |
| Vorne rechts                           | 135° | Vorwärts/Rückwärts                |
| Hinten links                           | 135° | Vorwärts/Rückwärts                |
| Hinten rehts                           | 45°  | Vorwärts/Rückwärts                |
|                                        |      |                                   |

# +
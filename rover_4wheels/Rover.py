
import socket
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
from enum import Enum

"""

TOPIC
{

}

"""

class DYNA_PARAM_UNIT(Enum):
    UNIT_RAW = 0,
    UNIT_PERCENT = 1,
    UNIT_RPM = 2,
    UNIT_DEGREE = 3,
    UNIT_MILLI_AMPERE = 4


class Rover(Node):
    """
    ROS2 Node zur Steuerung von Dynamixel-Motoren mit einem Logitech F710 Controller.
    - Der Node liest die Eingaben vom Controller, mappt diese auf einen Winkelbereich von 0 bis 300 Grad
      und sendet die entsprechenden Werte an den ESP32.
    """
    JS_LEFT_ROLL = 0
    JS_LEFT_PITCH = 1
    JS_RIGHT_ROLL = 4
    JS_RIGHT_PITCH = 3

    SERVO_01 = 2
    SERVO_02 = 7
    SERVO_03 = 16
    SERVO_04 = 18



    def __init__(self):
        super().__init__('ros2_esp32_dynamixel_with_controller')

        # Publisher für Motorbefehle
        self.motor_command_publisher = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info("Publisher 'motor_command' initialisiert")

        # ESP32-Verbindungsparameter
        self.esp32_ip = '192.168.0.199'  # Angepasste IP-Adresse des ESP32
        self.esp32_port = 12345          # Port des UDP-Servers auf dem ESP32

        # Initialisiere den Controller
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.get_logger().info(f"Controller erkannt: {self.controller.get_name()}")

        # Erstellt einen UDP-Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Letzte gesendete Positionen (zum Vergleich)
        self.last_positions = {18: None, 17: None, 16: None, 8: None}

    def listen_to_controller(self):
        """
        Hauptschleife des Nodes, die Controller-Eingaben verarbeitet und Motorbefehle sendet.
        """
        try:
            while rclpy.ok():
                #self.get_logger().info("-1-")
                rclpy.spin_once(self, timeout_sec=0.05)
                #self.get_logger().info("-2-")

                # Verarbeite Controller-Eingaben
                pygame.event.pump()
                #self.get_logger().info("-3-")

                # Joystick-Achsen lesen
                axis0 = self.controller.get_axis(self.JS_LEFT_ROLL)  # JS left ROLL
                axis1 = self.controller.get_axis(self.JS_LEFT_PITCH)  # JS left PITCH
                axis2 = self.controller.get_axis(self.JS_RIGHT_ROLL)  # JS right ROLL
                axis3 = self.controller.get_axis(self.JS_RIGHT_PITCH)  # JS right PITCH
                #self.get_logger().info("-4-")

                # Mappe Achsen auf Winkel
                motor_1_angle = self.map_value_to_angle(axis0)
                motor_2_angle = self.map_value_to_angle(axis1)
                motor_3_angle = self.map_value_to_angle(axis2)
                motor_4_angle = self.map_value_to_angle(axis3)
                #self.get_logger().info("-5-")

                # Sende nur bei Änderungen
                self.send_motor_command_if_changed(self.SERVO_01, motor_1_angle)
                self.send_motor_command_if_changed(self.SERVO_02, motor_2_angle)
                self.send_motor_command_if_changed(self.SERVO_03, motor_3_angle)
                self.send_motor_command_if_changed(self.SERVO_04, motor_4_angle)
                # self.send_motor_command_if_changed(2, motor_1_angle)
                # self.send_motor_command_if_changed(7, motor_2_angle)
                # self.send_motor_command_if_changed(16, motor_3_angle)
                # self.send_motor_command_if_changed(18, motor_4_angle)
                #self.get_logger().info("-6-")

                self.get_logger().debug(f"RAW0:{axis0:3.2f}\tRAW1:{axis1:3.2f}\tRAW2:{axis2:3.2f}\tRAW3:{axis3:3.2f}")
                self.get_logger().debug(f"S1({self.SERVO_01}):{motor_1_angle:3.2f}\tS2({self.SERVO_02}):{motor_2_angle:3.2f}\tS3({self.SERVO_03}):{motor_3_angle:3.2f}\tS4({self.SERVO_04}):{motor_4_angle:3.2f}")

        except Exception as e:
            self.get_logger().error(f"Fehler in der Hauptschleife: {e}")

    def map_value_to_angle(self, value, jitter=0.60):
        """
        Mappt den Wert des Controllers (-1.0 bis 1.0) auf den Bereich 0 bis 300 Grad.
        :param value: Eingabewert vom Controller (float zwischen -1.0 und 1.0)
        :param jitter: Threshold/Jitter rund um den Centerpoint.
        :return: Gemappter Winkel im Bereich 0 bis 300 Grad (float)
        """
        if jitter <= value <= jitter:
            return 150.0
        angle = (value + 1) * 150  # (+1, um den Bereich auf [0, 2] zu verschieben, dann *150)
        if angle <= (150.0 + jitter) and angle >= (150.0 - jitter):
            angle = 150.0
        return round(angle, 2)  # Winkel auf 2 Nachkommastellen runden

    def send_motor_command_if_changed(self, motor_id, angle):
        """
        Sendet einen Motorbefehl nur, wenn sich der Winkel geändert hat.
        :param motor_id: ID des Motors
        :param angle: Gemappter Winkel (float, 0 bis 300 Grad)
        """
        #self.get_logger().info("5.1")   
        if motor_id in self.last_positions:   
            if self.last_positions[motor_id] != angle:
                #self.get_logger().info("5.2a")      
                self.last_positions[motor_id] = angle
                #self.get_logger().info("5.2b")      
                self.get_logger().info(f"\n---\nMotorID:\t{motor_id:02d}\nAngle:\t{angle:3.2f}")
                self.send_motor_command(motor_id, angle)
                #self.get_logger().info("5.2c")      
        else:
            self.get_logger().info("5.3a")      
            self.last_positions[motor_id] = angle
            self.get_logger().info("5.3b")      
            self.send_motor_command(motor_id, angle)
            self.get_logger().info("5.3c")    

    def send_motor_command(self, motor_id, angle, unit=DYNA_PARAM_UNIT.UNIT_DEGREE):
        """
        Sendet einen Motorbefehl an den ESP32.
        :param motor_id: ID des Motors
        :param angle: Gemappter Winkel (float, 0 bis 300 Grad)
        """
        try:
            #self.get_logger().info("5.4a")      
            command = {
                "cmd": "move_motor",
                "id": motor_id,
                "position": angle,
                "unit" : ""
            }
            self.sock.sendto(json.dumps(command).encode('utf-8'), (self.esp32_ip, self.esp32_port))
            #self.get_logger().info("5.4b")      
        except Exception as e:
            self.get_logger().error(f"Fehler beim Senden des Motorbefehls: {e}")

    def close_connection(self):
        """
        Schließt die Verbindung zum ESP32 und gibt Ressourcen frei.
        """
        self.sock.close()
        self.get_logger().info("Verbindung zum ESP32 geschlossen.")

def main(args=None):
    """
    Haupteinstiegspunkt des ROS2-Nodes.
    """
    rclpy.init(args=args)
    node = Rover()

    try:
        node.listen_to_controller()
    except KeyboardInterrupt:
        pass
    finally:
        node.close_connection()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# import socket
# import json
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import pygame


# class ESP32DynamixelNodeWithController(Node):
#     """
#     ROS2 Node zur Steuerung von Dynamixel-Motoren mit einem Logitech F710 Controller.
#     - Der Node liest die Eingaben vom Controller, mappt diese auf einen Winkelbereich von 0 bis 300 Grad
#       und sendet die entsprechenden Werte an den ESP32.
#     """
#     JS_LEFT_ROLL = 0
#     JS_LEFT_PITCH = 1
#     JS_RIGHT_ROLL = 4
#     JS_RIGHT_PITCH = 3

#     def __init__(self):
#         super().__init__('ros2_esp32_dynamixel_with_controller')

#         # Publisher für Motorbefehle
#         self.motor_command_publisher = self.create_publisher(String, 'motor_command', 10)
#         self.get_logger().info("Publisher 'motor_command' initialisiert")

#         # ESP32-Verbindungsparameter
#         self.esp32_ip = '192.168.0.199'  # Angepasste IP-Adresse des ESP32
#         self.esp32_port = 12345          # Port des TCP-Servers auf dem ESP32

#         # Initialisiere den Controller
#         pygame.init()
#         pygame.joystick.init()
#         self.controller = pygame.joystick.Joystick(0)
#         self.controller.init()
#         self.get_logger().info(f"Controller erkannt: {self.controller.get_name()}")

#         # Erstellt einen TCP-Socket
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#         # Verbindet sich mit dem ESP32
#         try:
#             self.sock.connect((self.esp32_ip, self.esp32_port))
#             self.get_logger().info(f"Verbunden mit ESP32 bei {self.esp32_ip}:{self.esp32_port}")
#         except Exception as e:
#             self.get_logger().error(f"Fehler beim Verbinden mit ESP32: {e}")
#             self.destroy_node()
#             rclpy.shutdown()

#     def listen_to_controller(self):
#         """
#         Hauptschleife des Nodes, die Controller-Eingaben verarbeitet und Motorbefehle sendet.
#         """
#         try:
#             while rclpy.ok():
#                 rclpy.spin_once(self, timeout_sec=0.2)

#                 # Verarbeite Controller-Eingaben
#                 pygame.event.pump()

#                 # Joystick links (Motor 1)
#                 left_axis = self.controller.get_axis(1) # Wert von -32768 bis 32767
#                 axis0 = self.controller.get_axis(self.JS_LEFT_ROLL) # JS left ROLL
#                 axis1 = self.controller.get_axis(self.JS_LEFT_PITCH) # JS left PITCH
#                 axis2 = self.controller.get_axis(self.JS_RIGHT_ROLL) # JS right ROLL
#                 axis3 = self.controller.get_axis(self.JS_RIGHT_PITCH) # JS right PITCH

#                 axis4 = self.controller.get_axis(2) # LT
#                 axis5 = self.controller.get_axis(5) # RT

#                 motor_1_angle = self.map_value_to_angle(axis0)
#                 motor_2_angle = self.map_value_to_angle(axis1)
#                 motor_3_angle = self.map_value_to_angle(axis2)
#                 motor_4_angle = self.map_value_to_angle(axis3)

#                 self.send_motor_command(18, motor_1_angle)
#                 self.send_motor_command(17, motor_2_angle)
#                 self.send_motor_command(16, motor_3_angle)
#                 self.send_motor_command(8, motor_4_angle)

#                 self.get_logger().info(f"RAW0:{axis0:3.2f}\tRAW1:{axis1:3.2f}\tRAW2:{axis2:3.2f}\tRAW3:{axis3:3.2f}\tRAW4:{axis4:3.2f}\tRAW5:{axis5:3.2f}")
#                 self.get_logger().info(f"M1:{motor_1_angle:3.2f}\tM2:{motor_2_angle:3.2f}\tM3:{motor_3_angle:3.2f}\tM4:{motor_4_angle:3.2f}")


#         except Exception as e:
#             self.get_logger().error(f"Fehler in der Hauptschleife: {e}")

#     def map_value_to_angle(self, value, jitter=0.60):
#         """
#         Mappt den Wert des Controllers (-1.0 bis 1.0) auf den Bereich 0 bis 300 Grad.
#         :param value: Eingabewert vom Controller (float zwischen -1.0 und 1.0)
#         :param jitter: Threshold/Jitter rund um den Centerpoint.
#         :return: Gemappter Winkel im Bereich 0 bis 300 Grad (float)
#         """
#         # Mappt von [-1.0, 1.0] auf [0, 300]
#         if jitter <= value <= jitter:
#             return 150.0
#         angle = (value + 1) * 150  # (+1, um den Bereich auf [0, 2] zu verschieben, dann *150)
#         if angle <= (150.0+jitter) and angle >= (150.0-jitter):
#             #self.get_logger().info(f"mapped angle: {angle}")
#             angle = 150.0
#         return round(angle, 2)  # Winkel auf 2 Nachkommastellen runden

#     def send_motor_command(self, motor_id, angle):
#         """
#         Sendet einen Motorbefehl an den ESP32.
#         :param motor_id: ID des Motors
#         :param angle: Gemappter Winkel (float, 0 bis 300 Grad)
#         """
#         try:
#             command = {
#                 "command": "move_motor",
#                 "id": motor_id,
#                 "position": angle
#             }
#             self.sock.sendall((json.dumps(command) + '\n').encode('utf-8'))
#             #self.get_logger().info(f"Motor {motor_id} - Winkel: {angle}")
#         except Exception as e:
#             self.get_logger().error(f"Fehler beim Senden des Motorbefehls: {e}")

#     def close_connection(self):
#         """
#         Schließt die Verbindung zum ESP32 und gibt Ressourcen frei.
#         """
#         self.sock.close()
#         self.get_logger().info("Verbindung zum ESP32 geschlossen.")


# def main(args=None):
#     """
#     Haupteinstiegspunkt des ROS2-Nodes.
#     """
#     rclpy.init(args=args)
#     node = ESP32DynamixelNodeWithController()

#     try:
#         node.listen_to_controller()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.close_connection()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
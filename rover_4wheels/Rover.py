import socket
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from enum import Enum

class DYNA_PARAM_UNIT(Enum):
    UNIT_RAW = 0,
    UNIT_PERCENT = 1,
    UNIT_RPM = 2,
    UNIT_DEGREE = 3,
    UNIT_MILLI_AMPERE = 4


class Rover(Node):
    def __init__(self):
        super().__init__('rover_joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.get_logger().info("rover_joy_subscriber erstellt....")

        self.subscription  # verhindert, dass Garbage Collector die Subscription entfernt
        self.get_logger().info("Rover Node gestartet, wartet auf Joy-Nachrichten...")

        # Publisher für Motorbefehle
        # Bitte bedenken, das hier ist jetzt kein ROS2 Publisher !!!!!
        # Dieser Publisher fungiert nur um mit dem ESP32 über die Schnittstelle zu arbeiten

        self.motor_command_publisher = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info("Publisher 'motor_command' initialisiert")

        # ESP32-Verbindungsparameter
        self.esp32_ip = '192.168.0.199'  # Angepasste IP-Adresse des ESP32
        self.esp32_port = 12345          # Port des UDP-Servers auf dem ESP32

        self.subscription  # verhindert, dass Garbage Collector die Subscription entfernt
        self.get_logger().info("Rover Node gestartet, wartet auf Joy-Nachrichten...")


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
                rclpy.spin_once(self, timeout_sec=0.05)
                
                
        except Exception as e:
            self.get_logger().error(f"Fehler in der Hauptschleife: {e}")

    def joy_callback(self, msg):
        # Hier kannst du die Joy-Daten auswerten und nutzen
        self.get_logger().info(f"Empfangene Joy-Daten: {msg.axes}, {msg.buttons}")

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

import socket
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from enum import Enum, auto


""" 

******************************************************************
JOY-Stick Subscriber
******************************************************************

axes[0]: Linker Stick horizontal (links = -1.0, rechts = 1.0)
axes[1]: Linker Stick vertikal (unten = -1.0, oben = 1.0)
axes[2]: Rechter Stick horizontal
axes[3]: Rechter Stick vertikal
axes[4]: L2-Trigger (0 bis 1)
axes[5]: R2-Trigger (0 bis 1)

buttons[0]: A-Knopf (0 = nicht gedrückt, 1 = gedrückt)
buttons[1]: B-Knopf
buttons[2]: X-Knopf
buttons[3]: Y-Knopf
buttons[4]: LB (Left Bumper)
buttons[5]: RB (Right Bumper)
buttons[6]: Back-Taste
buttons[7]: Start-Taste
buttons[8]: Stick-Button links
buttons[9]: Stick-Button rechts


"""

class DYNA_PARAM_UNIT(Enum):
    UNIT_RAW = 0,
    UNIT_PERCENT = 1,
    UNIT_RPM = 2,
    UNIT_DEGREE = 3,
    UNIT_MILLI_AMPERE = 4


class DYNA_CTRL_TABLE(Enum):
    MODEL_NUMBER = 0,
    MODEL_INFORMATION = auto(),
    FIRMWARE_VERSION= auto(),
    PROTOCOL_VERSION= auto(),
    ID= auto(),
    SECONDARY_ID= auto(),
    BAUD_RATE= auto(),
    DRIVE_MODE= auto(),
    CONTROL_MODE= auto(),
    OPERATING_MODE= auto(),
    CW_ANGLE_LIMIT= auto(),
    CCW_ANGLE_LIMIT= auto(),
    TEMPERATURE_LIMIT= auto(),
    MIN_VOLTAGE_LIMIT= auto(),
    MAX_VOLTAGE_LIMIT= auto(),
    PWM_LIMIT= auto(),
    CURRENT_LIMIT= auto(),
    VELOCITY_LIMIT= auto(),
    MAX_POSITION_LIMIT= auto(),
    MIN_POSITION_LIMIT= auto(),
    ACCELERATION_LIMIT= auto(),
    MAX_TORQUE= auto(),
    HOMING_OFFSET= auto(),
    MOVING_THRESHOLD= auto(),
    MULTI_TURN_OFFSET= auto(),
    RESOLUTION_DIVIDER= auto(),
    EXTERNAL_PORT_MODE_1= auto(),
    EXTERNAL_PORT_MODE_2= auto(),
    EXTERNAL_PORT_MODE_3= auto(),
    EXTERNAL_PORT_MODE_4= auto(),
    STATUS_RETURN_LEVEL= auto(),
    RETURN_DELAY_TIME= auto(),
    ALARM_LED= auto(),
    SHUTDOWN= auto(),

    TORQUE_ENABLE= auto(),
    LED= auto(),
    LED_RED= auto(),
    LED_GREEN= auto(),
    LED_BLUE= auto(),
    REGISTERED_INSTRUCTION= auto(),
    HARDWARE_ERROR_STATUS= auto(),
    VELOCITY_P_GAIN= auto(),
    VELOCITY_I_GAIN= auto(),
    POSITION_P_GAIN= auto(),
    POSITION_I_GAIN= auto(),
    POSITION_D_GAIN= auto(),
    FEEDFORWARD_1ST_GAIN= auto(),
    FEEDFORWARD_2ND_GAIN= auto(),
    P_GAIN= auto(),
    I_GAIN= auto(),
    D_GAIN= auto(),
    CW_COMPLIANCE_MARGIN= auto(),
    CCW_COMPLIANCE_MARGIN= auto(),
    CW_COMPLIANCE_SLOPE= auto(),
    CCW_COMPLIANCE_SLOPE= auto(),
    GOAL_PWM= auto(),
    GOAL_TORQUE= auto(),
    GOAL_CURRENT= auto(),
    GOAL_POSITION= auto(),
    GOAL_VELOCITY= auto(),
    GOAL_ACCELERATION= auto(),
    MOVING_SPEED= auto(),
    PRESENT_PWM= auto(),
    PRESENT_LOAD= auto(),
    PRESENT_SPEED= auto(),
    PRESENT_CURRENT= auto(),
    PRESENT_POSITION= auto(),
    PRESENT_VELOCITY= auto(),
    PRESENT_VOLTAGE= auto(),
    PRESENT_TEMPERATURE= auto(),
    TORQUE_LIMIT= auto(),
    REGISTERED= auto(),
    MOVING= auto(),
    LOCK= auto(),
    PUNCH= auto(),
    CURRENT= auto(),
    SENSED_CURRENT= auto(),
    REALTIME_TICK= auto(),
    TORQUE_CTRL_MODE_ENABLE= auto(),
    BUS_WATCHDOG= auto(),
    PROFILE_ACCELERATION= auto(),
    PROFILE_VELOCITY= auto(),
    MOVING_STATUS= auto(),
    VELOCITY_TRAJECTORY= auto(),
    POSITION_TRAJECTORY= auto(),
    PRESENT_INPUT_VOLTAGE= auto(),
    EXTERNAL_PORT_DATA_1= auto(),
    EXTERNAL_PORT_DATA_2= auto(),
    EXTERNAL_PORT_DATA_3= auto(),
    EXTERNAL_PORT_DATA_4= auto(),

    # for DY
    STARTUP_CONFIGURATION= auto(),
    POSITION_LIMIT_THRESHOLD= auto(),
    IN_POSITION_THRESHOLD= auto(),
    FOLLOWING_ERROR_THRESHOLD= auto(),
    INVERTER_TEMPERATURE_LIMIT= auto(),
    MOTOR_TEMPERATURE_LIMIT= auto(),
    ELECTRONIC_GEAR_RATIO_NUMERATOR= auto(),
    ELECTRONIC_GEAR_RATIO_DENOMINATOR= auto(),
    Safe_STOP_TIME= auto(),
    BRAKE_DELAY= auto(),
    GOAL_UPDATE_DELAY= auto(),
    OVEREXCITATION_VOLTAGE= auto(),
    NORMAL_EXCITATION_VOLTAGE= auto(),
    OVEREXCITATION_TIME= auto(),
    PRESENT_VELOCITY_LPF_FREQUENCY= auto(),
    GOAL_CURRENT_LPF_FREQUENCY= auto(),
    POSITION_FF_LPF_TIME= auto(),
    VELOCITY_FF_LPF_TIME= auto(),
    CONTROLLER_STATE= auto(),
    ERROR_CODE= auto(),
    ERROR_CODE_HISTORY1= auto(),
    HYBRID_SAVE= auto(),
    PROFILE_ACCELERATION_TIME= auto(),
    PROFIIE_TIME= auto(),
    PWM_OFFSET= auto(),
    CURRENT_OFFSET= auto(),
    VELOCITY_OFFSET= auto(),
    PRESENT_MOTOR_TEMPERATURE= auto(),


    LAST_DUMMY_ITEM = 0xFF


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

        self.msg = None

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
        #
        # Wird immer aufgerufen, wenn vom Publisher eine neue Nachricht versendet wird
        #
        # msg.header.stamp  # Zeitstempel der Nachricht
        # msg.axes = [0.0, -1.0, 0.5, 0.0, -0.5, 1.0]
        # msg.buttons = [0, 1, 0, 0, 1, 0, 0, 0, 0, 0]
        #
        self.get_logger().info(f"Empfangene Joy-Daten: {msg.axes}, {msg.buttons}")
        self.msg = msg

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

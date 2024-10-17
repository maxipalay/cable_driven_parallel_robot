import rclpy
from rclpy.node import Node
import serial

from cdpr_kinematics_interfaces.msg import JointCommand

class CDPRHardware(Node):
    def __init__(self):
        super().__init__("CPDR_driver")
        self.serial = None
        if not self.initialize_serial():
            self.destroy_node()
            exit()
        
        self.create_subscription(JointCommand, "joint_commands", self.cb_joint_command, 10)
        #self.homing()


    def initialize_serial(self):
        try:
            self.serial = serial.Serial('/dev/ttyACM0', baudrate=115200)
        except Exception as e:
            self.get_logger().error(str(e))
            return False
        return True
    
    def cb_joint_command(self, msg):
        self.get_logger().info(f"sending lengths: {msg.cable1_length:.2f}, {msg.cable2_length:.2f}, {msg.cable3_length:.2f}, {msg.cable4_length:.2f}")
        self.serial.write(f"m {msg.cable1_length} {msg.cable2_length} {msg.cable3_length} {msg.cable4_length}\\n".encode())
        return

    def homing(self):
        self.get_logger().info('homing')
        self.serial.write(b'h\n')
        answer = self.serial.readline().decode()
        self.get_logger().info(answer)
        return
    
def main():
    rclpy.init()
    node = CDPRHardware()
    rclpy.spin(node)
    rclpy.shutdown()
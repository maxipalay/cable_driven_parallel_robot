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
        self.create_timer(0.025, self.timer_cb)
        self.pub_feedback = self.create_publisher(JointCommand, "cable_lengths", 10)
        #self.homing()

        # Lambda function to convert the incoming string to an array of floats
        self.convert_to_floats = lambda s: list(map(float, s[1:].strip().split(',')))
        self.cable_lengths = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]

    def timer_cb(self,):
        if self.serial is not None:
            recv_str = self.serial.readline().decode()
            if recv_str != "":
                if recv_str[0] == "f":
                    try:
                        self.cable_lengths = self.convert_to_floats(recv_str)
                    except Exception as e:
                        self.get_logger().debug(str(e))
                        return
                    msg = JointCommand(cable_lengths = self.cable_lengths, cable_tensions = [])
                    self.pub_feedback.publish(msg)

    def initialize_serial(self):
        try:
            self.serial = serial.Serial('/dev/ttyACM0', baudrate=250000, timeout=0.0)
        except Exception as e:
            self.get_logger().error(str(e))
            return False
        return True
    
    def cb_joint_command(self, msg):
        self.get_logger().info(f"sending lengths: {msg.cable1_length:.4f}, {msg.cable2_length:.4f}, {msg.cable3_length:.4f}, {msg.cable4_length:.4f}, {msg.cable5_length:.4f}, {msg.cable6_length:.4f}, {msg.cable7_length:.4f}, {msg.cable8_length:.4f}")
        self.serial.write(f"m {msg.cable1_length} {msg.cable2_length} {msg.cable3_length} {msg.cable4_length} {msg.cable5_length} {msg.cable6_length} {msg.cable7_length} {msg.cable8_length}\\n".encode())
        self.serial.flush()
        return

    def homing(self):
        self.get_logger().info('homing')
        self.serial.write(b'h\n')
        # flush?
        #answer = self.serial.readline().decode()
        #self.get_logger().info(answer)
        return
    
def main():
    rclpy.init()
    node = CDPRHardware()
    rclpy.spin(node)
    rclpy.shutdown()
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
        self.create_timer(0.01, self.timer_cb)
        self.pub_feedback = self.create_publisher(JointCommand, "cable_lengths", 10)

        # Lambda function to convert the incoming string to an array of floats
        self.convert_to_floats = lambda s: list(map(float, s[2:].strip().split(',')))
        self.cable_lengths = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
        self.torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.new_data = False

    def timer_cb(self,):
        if self.serial is not None:
            recv_str = None
            while recv_str != "":
                recv_str = self.serial.readline().decode()
                if recv_str != "":
                    if recv_str[0:2] == "fp":
                        try:
                            self.cable_lengths = self.convert_to_floats(recv_str)
                        except Exception as e:
                            self.get_logger().debug(str(e))
                            return
                        self.new_data = True
                    elif recv_str[0:2] == "ft":
                        try:
                            self.torques = self.convert_to_floats(recv_str)
                        except Exception as e:
                            self.get_logger().debug(str(e))
                            return
                        self.new_data = True            
        if self.new_data:
            msg = JointCommand(cable_lengths= self.cable_lengths, cable_tensions = self.torques)
            self.pub_feedback.publish(msg)
            self.new_data = False

    def initialize_serial(self):
        try:
            self.serial = serial.Serial('/dev/ttyACM0', baudrate=500000, timeout=0.0)
            self.serial.set_low_latency_mode(True)
        except Exception as e:
            self.get_logger().error(str(e))
            return False
        return True
    
    def cb_joint_command(self, msg):
        self.get_logger().info(f"sending lengths: {msg.cable_lengths[0]:.4f}, {msg.cable_lengths[1]:.4f}, {msg.cable_lengths[2]:.4f}, {msg.cable_lengths[3]:.4f}, {msg.cable_lengths[4]:.4f}, {msg.cable_lengths[5]:.4f}, {msg.cable_lengths[6]:.4f}, {msg.cable_lengths[7]:.4f}")
        self.get_logger().info(f"sending tensions: {msg.cable_tensions[0]:.4f}, {msg.cable_tensions[1]:.4f}, {msg.cable_tensions[2]:.4f}, {msg.cable_tensions[3]:.4f}, {msg.cable_tensions[4]:.4f}, {msg.cable_tensions[5]:.4f}, {msg.cable_tensions[6]:.4f}, {msg.cable_tensions[7]:.4f}")
        self.serial.write(f"t {msg.cable_lengths[0]:.6f} {msg.cable_lengths[1]:.6f} {msg.cable_lengths[2]:.6f} {msg.cable_lengths[3]:.6f} {msg.cable_lengths[4]:.6f} {msg.cable_lengths[5]:.6f} {msg.cable_lengths[6]:.6f} {msg.cable_lengths[7]:.6f} {msg.cable_tensions[0]:.6f} {msg.cable_tensions[1]:.6f} {msg.cable_tensions[2]:.6f} {msg.cable_tensions[3]:.6f} {msg.cable_tensions[4]:.6f} {msg.cable_tensions[5]:.6f} {msg.cable_tensions[6]:.6f} {msg.cable_tensions[7]:.6f}\\n".encode())
        self.serial.flush()
        return
    
def main():
    rclpy.init()
    node = CDPRHardware()
    rclpy.spin(node)
    rclpy.shutdown()
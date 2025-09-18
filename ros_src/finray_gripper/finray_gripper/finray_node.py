
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
import can 
import struct



class FinrayGripper(Node):

    def __init__(self):
        super().__init__('finray_gripper')
        #odrive params
        self.node_id = 6
        self.axis = 0

        #operation params
        self.vel_limit = 3.0 #[rev/s]
        self.torque_ff = 0.0
        self.publisher_rate = 10 #[hz]
        self.current_pos = 0.0
        self.torq_threshold = 0.75 #[Nm]
        self.current_threshold = 8.0 #[A]
        self.motor_kt = 0.083037 #[Nm/A]

        #homing params
        self.homing_vel = 2.0 #[rev/s]
        self.homing_torq = 0.75 #[Nm]
        self.homed = False
        self.home_pos = 0.0

        #
        

        self.bus  = can.interface.Bus(channel='can0', bustype='socketcan')


        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def homing(self):
        velocity = 0


def main(args=None):
    rclpy.init(args=args)

    finray = FinrayGripper()

    rclpy.spin(finray)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finray.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

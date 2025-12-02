#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion
import math

class IMUDeadReckoner:
    # state for a single IMU
    def __init__(self):
        self.reset()
    def reset(self):
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.data = None

class DeadReckoningNode(Node): # for all 3 IMUs
    def __init__(self):
        super().__init__('dead_reckoning')
        self.imu_dr = {
            "imu1": IMUDeadReckoner(),
            "imu2": IMUDeadReckoner(),
            "imu3": IMUDeadReckoner()
        }
        self.create_subscription(Float32MultiArray, 'microROS/imu1', self.cb_imu1, 10)
        self.create_subscription(Float32MultiArray, 'microROS/imu2', self.cb_imu2, 10)
        self.create_subscription(Float32MultiArray, 'microROS/imu3', self.cb_imu3, 10)
        self.pub = {
            "imu1": self.create_publisher(Pose, 'dead_reckoning/imu1/pose', 10), # I think if we use Pose, we can visualize in rviz, can also try plotjuggler
            "imu2": self.create_publisher(Pose, 'dead_reckoning/imu2/pose', 10),
            "imu3": self.create_publisher(Pose, 'dead_reckoning/imu3/pose', 10)
        }

        self.dt = 0.01 # 100 Hz
        self.timer = self.create_timer(self.dt, self.update_all)

        self.get_logger().info("Dead Reckoning for 3 IMUs started.")

    def cb_imu1(self, msg): self.imu_dr["imu1"].data = msg.data
    def cb_imu2(self, msg): self.imu_dr["imu2"].data = msg.data
    def cb_imu3(self, msg): self.imu_dr["imu3"].data = msg.data

    def update_all(self):
        for key in ["imu1", "imu2", "imu3"]:
            dr = self.imu_dr[key]
            if dr.data is None:
                continue
            self.update_single(dr)
            # print(f"KEY: {self.pub[key]}")
            self.publish_pose(dr, self.pub[key])

    def update_single(self, dr: IMUDeadReckoner):
        ax, ay, az, roll, pitch, yaw = dr.data
        dr.roll = roll
        dr.pitch = pitch
        dr.yaw = yaw

        # rotation matrices for body to world
        cr = math.cos(dr.roll)
        sr = math.sin(dr.roll)
        cp = math.cos(dr.pitch)
        sp = math.sin(dr.pitch)
        cy = math.cos(dr.yaw)
        sy = math.sin(dr.yaw)

        # convert acceleration to world frame
        ax_w = ax * (cp * cy) + ay * (sr * sp * cy - cr * sy) + az * (cr * sp * cy + sr * sy)
        ay_w = ax * (cp * sy) + ay * (sr * sp * sy + cr * cy) + az * (cr * sp * sy - sr * cy)
        az_w = ax * (-sp)     + ay * (sr * cp)              + az * (cr * cp)

        # integrate velocity
        dr.vx += ax_w * self.dt
        dr.vy += ay_w * self.dt
        dr.vz += az_w * self.dt

        # integrate position
        dr.x += dr.vx * self.dt
        dr.y += dr.vy * self.dt
        dr.z += dr.vz * self.dt

    def publish_pose(self, dr: IMUDeadReckoner, pub):
        # print(f"dr: {dr.x}")
        pose = Pose()
        [pose.position.x, pose.position.y, pose.position.z] = [dr.x, dr.y, dr.z]
        # convert roll/pitch/yaw to quaternion, for Pose message
        cy = math.cos(dr.yaw * 0.5)
        sy = math.sin(dr.yaw * 0.5)
        cp = math.cos(dr.pitch * 0.5)
        sp = math.sin(dr.pitch * 0.5)
        cr = math.cos(dr.roll * 0.5)
        sr = math.sin(dr.roll * 0.5)
        pose.orientation = Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )
        pub.publish(pose)
        # print(pose)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores, register_types 
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Path to your bag
bag_path = Path("/home/jn2/college/Fin_Ray_Gripper/data/rosbag2_2025_11_20-19_30_53/rosbag2_2025_11_20-19_30_53_0.db3")

topic = "/display_planned_path"


# Typestore for ROS 2 Humble
typestore = get_typestore(Stores.ROS2_HUMBLE)


def load_all_needed_msgdefs():
    """
    Build a dict of msg_name -> msg_text for all message types
    required by DisplayTrajectory.
    """

    msgdefs = {}

    # builtin
    msgdefs["builtin_interfaces/msg/Time"] = """
int32 sec
uint32 nanosec
"""

    # std_msgs
    msgdefs["std_msgs/msg/Header"] = """
builtin_interfaces/msg/Time stamp
string frame_id
uint32 seq
"""

    # trajectory_msgs
    msgdefs["trajectory_msgs/msg/JointTrajectoryPoint"] = """
sequence<double> positions
sequence<double> velocities
sequence<double> accelerations
sequence<double> effort
builtin_interfaces/msg/Duration time_from_start
"""

    msgdefs["builtin_interfaces/msg/Duration"] = """
int32 sec
uint32 nanosec
"""

    msgdefs["trajectory_msgs/msg/JointTrajectory"] = """
std_msgs/msg/Header header
sequence<string> joint_names
sequence<trajectory_msgs/msg/JointTrajectoryPoint> points
"""

    # moveit core messages
    msgdefs["moveit_msgs/msg/RobotState"] = """
sequence<string> joint_state_name
sequence<double> joint_state_position
sequence<double> joint_state_velocity
sequence<double> joint_state_effort
"""

    msgdefs["moveit_msgs/msg/RobotTrajectory"] = """
trajectory_msgs/msg/JointTrajectory joint_trajectory
trajectory_msgs/msg/MultiDOFJointTrajectory multi_dof_joint_trajectory
"""

    msgdefs["trajectory_msgs/msg/MultiDOFJointTrajectory"] = """
std_msgs/msg/Header header
sequence<string> joint_names
sequence<trajectory_msgs/msg/MultiDOFJointTrajectoryPoint> points
"""

    msgdefs["trajectory_msgs/msg/MultiDOFJointTrajectoryPoint"] = """
sequence<geometry_msgs/msg/Transform> transforms
sequence<geometry_msgs/msg/Twist> velocities
sequence<geometry_msgs/msg/Twist> accelerations
builtin_interfaces/msg/Duration time_from_start
"""

    # geometry_msgs
    msgdefs["geometry_msgs/msg/Transform"] = """
geometry_msgs/msg/Vector3 translation
geometry_msgs/msg/Quaternion rotation
"""

    msgdefs["geometry_msgs/msg/Vector3"] = """
double x
double y
double z
"""

    msgdefs["geometry_msgs/msg/Quaternion"] = """
double x
double y
double z
double w
"""

    msgdefs["geometry_msgs/msg/Twist"] = """
geometry_msgs/msg/Vector3 linear
geometry_msgs/msg/Vector3 angular
"""

    # The message you need
    msgdefs["moveit_msgs/msg/DisplayTrajectory"] = """
moveit_msgs/msg/RobotState model_id
sequence<moveit_msgs/msg/RobotTrajectory> trajectory
"""

    return msgdefs

custom_messages = load_all_needed_msgdefs()
for msg_name, msg_def in custom_messages.items():
    typestore.add_msgdef(msg_name, msg_def)



with AnyReader([bag_path], default_typestore=typestore) as reader:
    # Find only messages on this topic
    connections = [x for x in reader.connections if x.topic == topic]
    msgs = list(reader.messages(connections=connections))

times = []
velocities = []   # list of N×6 velocity vectors
joint_names = None

for conn, timestamp, rawdata in msgs:
    msg = reader.deserialize(rawdata, conn.msgtype)

    # msg is moveit_msgs/DisplayTrajectory
    # It may contain multiple trajectories, so iterate everything
    for traj in msg.trajectory:
        jt = traj.joint_trajectory

        if joint_names is None:
            joint_names = list(jt.joint_names)

        for pt in jt.points:
            velocities.append(pt.velocities)
            times.append(timestamp * 1e-9)

# Convert to arrays
velocities = np.array(velocities)
times = np.array(times)
rel_time = times - times[0]

# ----- Plot -----

plt.figure(figsize=(12, 6))
plt.title("Joint Velocities from MoveIt /display_planned_path")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [rad/s]")

for i, name in enumerate(joint_names):
    plt.plot(rel_time, velocities[:, i], label=name)

plt.legend(loc="upper right")
plt.grid(True)
plt.tight_layout()
plt.show()

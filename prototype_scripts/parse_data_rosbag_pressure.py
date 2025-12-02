from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import cv2

#Joe's cell IMG_3197.MOV
#Apple 101 @ 13:22 Batch 199 Apple 1

bag_path = Path("/home/jn2/college/Fin_Ray_Gripper/data/batch_65/apple_0/final_approach_and_pick_20251029_125029.db3/final_approach_and_pick_20251029_125029.db3_0.db3")

topic = "/microROS/sensor_data"

typestore = get_typestore(Stores.ROS2_HUMBLE)  # or "ros2-humble" depending on your distro

with AnyReader([bag_path], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == topic]
    msgs = list(reader.messages(connections=connections))

times = []
values = []
for conn, timestamp, rawdata in msgs:
    msg = reader.deserialize(rawdata, conn.msgtype)
    values.append(msg.data)
    times.append(timestamp * 1e-9)

pressure = []
tof = []
rel_time = np.array(times) - times[0]

for x in values:
    pressure.append(x[:3])
    tof.append(x[-1])

plt.figure()
plt.title(f"Pressure Sensors")
plt.xlabel("Time [s]")
plt.ylabel("Pressure [hPa]")
plt.plot(rel_time, np.array(pressure))
plt.legend(["Sensor 1", "Sensor 2", "Sensor 3"], loc="upper right")

plt.figure()
plt.title(f"TOF Sensors")
plt.xlabel("Time [s]")
plt.ylabel("Distance [mm]")
plt.plot(rel_time, np.array(tof))



bag_path = Path("/home/jn2/college/Fin_Ray_Gripper/data/batch_199/apple_0/visual_servo_20251030_131816.db3/visual_servo_20251030_131816.db3_0.db3")
bag_path = Path("/home/jn2/college/Fin_Ray_Gripper/data/batch_65/apple_0/visual_servo_20251029_125022.db3/visual_servo_20251029_125022.db3_0.db3")
topic = "/gripper/rgb_palm_camera/image_raw"


with AnyReader([bag_path], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == topic]
    msgs = list(reader.messages(connections=connections))


# Prepare output video

duration = 6117924503 #nanosec
num_messages = 1562
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
fps = num_messages/(duration * 1E-9)  # adjust if you know your frame rate
print(fps)
out = None

for conn, timestamp, rawdata in msgs:
    msg = reader.deserialize(rawdata, conn.msgtype)

    # Convert ROS Image -> NumPy array
    dtype = np.uint8
    img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, -1)

    # Initialize video writer after knowing size
    if out is None:
        out = cv2.VideoWriter("/home/jn2/college/Fin_Ray_Gripper/data/output.mp4", fourcc, fps, (msg.width, msg.height))

    # Convert color if needed
    if msg.encoding in ("rgb8", "RGB8"):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding in ("mono8",):
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    out.write(img)

out.release()
print("✅ Video saved as output.mp4")
plt.show()

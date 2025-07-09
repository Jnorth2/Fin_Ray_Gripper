import serial
import matplotlib.pyplot as plt
from collections import deque
import re
import time
import keyboard  # For pause/resume with keys
import pandas as pd

# === SERIAL SETUP ===
ser = serial.Serial('COM7', 115200, timeout=1)  # Set your COM port
ser.reset_input_buffer()  # Clear buffer on startup

class IMU_Data: 
    def __init__(self, imu_id=0, N=200):
        self.imu = imu_id
        self.timestamps = deque(maxlen=N)
        self.roll = deque(maxlen=N)
        self.pitch = deque(maxlen=N)
        self.yaw = deque(maxlen=N)
        self.ax = deque(maxlen=N)
        self.ay = deque(maxlen=N)
        self.az = deque(maxlen=N)
        self.gx = deque(maxlen=N)
        self.gy = deque(maxlen=N)
        self.gz = deque(maxlen=N)

    def clear(self):
        self.timestamps.clear()
        self.roll.clear()
        self.pitch.clear()
        self.yaw.clear()
        self.ax.clear()
        self.ay.clear()
        self.az.clear()
        self.gx.clear()
        self.gy.clear()
        self.gz.clear()
    
    def append_data(self, timestamp, roll, pitch, yaw, ax, ay, az, gx, gy, gz):
        self.timestamps.append(timestamp)
        self.roll.append(roll)
        self.pitch.append(pitch)
        self.yaw.append(yaw)
        self.ax.append(ax)
        self.ay.append(ay)
        self.az.append(az)
        self.gx.append(gx)
        self.gy.append(gy)
        self.gz.append(gz)


# === DATA BUFFERS ===
N = 30 * 50 # 30 seconds at 50Hz
IMU1_Data = IMU_Data(1, N)
IMU2_Data = IMU_Data(2, N)
data = []
data2 = []

# === PLOTTING SETUP ===
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

# Orientation plot
line_roll, = ax1.plot([], [], label='Roll')
line_pitch, = ax1.plot([], [], label='Pitch')
line_yaw, = ax1.plot([], [], label='Yaw')
ax1.set_title("Orientation (Euler angles)")
ax1.set_ylabel("Degrees")
ax1.legend()
ax1.grid(True)

# Acceleration plot
line_ax, = ax2.plot([], [], label='Ax')
line_ay, = ax2.plot([], [], label='Ay')
line_az, = ax2.plot([], [], label='Az')
ax2.set_title("Acceleration")
ax2.set_ylabel("m/s²")
ax2.set_xlabel("Time (seconds)")
ax2.legend()
ax2.grid(True)

# Regex pattern for labeled serial data
# IMU_ID TIMESTAMP ROLL PITCH YAW AX AY AZ GX GY GZ
pattern = re.compile(
    r'(\d+)\s*,\s*'          # IMU_ID
    r'(\d+)\s*,\s*'          # Timestamp
    r'([-+]?\d*\.\d+|\d+)\s*,\s*'  # Yaw
    r'([-+]?\d*\.\d+|\d+)\s*,\s*'  # Pitch
    r'([-+]?\d*\.\d+|\d+)\s*,\s*'  # Roll
    r'([-+]?\d*\.\d+|\d+)\s*,\s*'  # Ax
    r'([-+]?\d*\.\d+|\d+)\s*,\s*'  # Ay
    r'([-+]?\d*\.\d+|\d+)\s*,\s*'  # Az
    r'([-+]?\d*\.\d+|\d+)\s*,\s*'  # Gx
    r'([-+]?\d*\.\d+|\d+)\s*,\s*'  # Gy
    r'([-+]?\d*\.\d+|\d+)'           # Gz (no trailing comma)
)


# === MAIN LOOP ===
frame = 0
plot_every_n = 10
paused = True
saved = False

start_time = time.time()  # Record start time for real-time calculation

while True:
    try:
        if keyboard.is_pressed('p') and not paused:
            paused = True
            time.sleep(0.3)
            print("Paused — press 'r' to resume")
        #record data    
        if paused and keyboard.is_pressed('o') and not saved: 
            data.clear()
            data = {
                'Time': IMU1_Data.timestamps,
                'Roll': IMU1_Data.roll,
                'Pitch': IMU1_Data.pitch,
                'Yaw': IMU1_Data.yaw,
                'Ax': IMU1_Data.ax,
                'Ay': IMU1_Data.ay,
                'Az': IMU1_Data.az,
                'Gx': IMU1_Data.gx,
                'Gy': IMU1_Data.gy,
                'Gz': IMU1_Data.gz,
            }
            df = pd.DataFrame(data)
            df.to_csv('imu1_test_rec.csv', index=False)
            print("Saved IMU1 Data!")
            data2 = {
                'Time': IMU2_Data.timestamps,
                'Roll': IMU2_Data.roll,
                'Pitch': IMU2_Data.pitch,
                'Yaw': IMU2_Data.yaw,
                'Ax': IMU2_Data.ax,
                'Ay': IMU2_Data.ay,
                'Az': IMU2_Data.az,
                'Gx': IMU2_Data.gx,
                'Gy': IMU2_Data.gy,
                'Gz': IMU2_Data.gz,
            }
            df2 = pd.DataFrame(data2)
            df2.to_csv('imu2_test_rec.csv', index=False)
            print("Saved IMU2 Data!")
            saved = True

            time.sleep(0.3)

        if keyboard.is_pressed('r') and paused:
            paused = False
            saved = False
            IMU1_Data.clear()  # Clear previous data
            IMU2_Data.clear()  # Clear previous data
            time.sleep(0.3)
            start_time = time.time()
            print("Resumed")

        if keyboard.is_pressed('q'):
            print("Exiting...")
            break

        if paused:
            plt.pause(0.1)
            continue

        line_raw = ser.readline().decode(errors='ignore').strip()
        match = pattern.match(line_raw)
        if not match:
            continue

        imu = float(match.group(1))
        timestamp = float(match.group(2))
        yaw = float(match.group(3))
        pitch = float(match.group(4))
        roll = float(match.group(5))
        ax = float(match.group(6))
        ay = float(match.group(7))
        az = float(match.group(8))
        gx = float(match.group(9)) # Gyroscope data
        gy = float(match.group(10))
        gz = float(match.group(11))

        # Store data in IMU1_Data
        if imu == IMU1_Data.imu:
            IMU1_Data.append_data(timestamp, roll, pitch, yaw, ax, ay, az, gx, gy, gz)
        elif imu == IMU2_Data.imu:
            IMU2_Data.append_data(timestamp, roll, pitch, yaw, ax, ay, az, gx, gy, gz)

        frame += 1
        if frame % plot_every_n == 0:
            # Update orientation plot
            line_roll.set_data(IMU1_Data.timestamps, IMU1_Data.roll)
            line_pitch.set_data(IMU1_Data.timestamps, IMU1_Data.pitch)
            line_yaw.set_data(IMU1_Data.timestamps, IMU1_Data.yaw)
            ax1.relim()
            ax1.autoscale_view()

            # Update acceleration plot
            line_ax.set_data(IMU1_Data.timestamps, IMU1_Data.ax)
            line_ay.set_data(IMU1_Data.timestamps, IMU1_Data.ay)
            line_az.set_data(IMU1_Data.timestamps, IMU1_Data.az)
            ax2.relim()
            ax2.autoscale_view()

            plt.pause(0.001)

    except Exception as e:
        print("Error:", e)

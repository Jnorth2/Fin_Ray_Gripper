import serial


ser = serial.Serial('COM7', 115200, timeout=1)  # Set your COM port
ser.reset_input_buffer()  # Clear buffer on startup

while True:
    line = ser.readline().decode(errors='ignore').strip()
    if line:
        print(line)

import serial
import time

PORT = "/dev/ttyACM0"   # change if needed
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print("Connected to Arduino")

try:
    while True:
        # Send velocity command
        cmd = input("Enter: ")
        ser.write((cmd+"\n").encode())

        # Read response
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(line)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping")
    ser.write(b"V 0.0 0.0\n")
    time.sleep(0.1)
    ser.close()

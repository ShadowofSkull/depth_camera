import serial
import time

# Initialize serial connection
ser = serial.Serial("COM4", 115200, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

# Send data to Arduino
test_data = "forward\n"


# Read response from Arduino
while True:
    print(f"Sending: {test_data.strip()}")  # Debug print
    ser.write(test_data.encode("utf-8"))  # Encode the data before sending
    if ser.in_waiting > 0:
        line = ser.readline().decode("utf-8").rstrip()  # Decode the received data
        print(f"Received: {line}")  # Debug print
    time.sleep(1)

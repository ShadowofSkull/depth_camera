import serial
import time

# Initialize serial connection
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2) # Wait for the serial connection to initialize

# Send data to Arduino
test_data = 'Hello, Arduino!\n'
print(f"Sending: {test_data.strip()}") # Debug print
ser.write(test_data.encode('utf-8')) # Encode the data before sending

# Read response from Arduino
while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip() # Decode the received data
        print(f"Received: {line}") # Debug print
        break # Exit loop after receiving the response

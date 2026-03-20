import serial
import time

ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

ser.write(b'T 15000\n')

with open('step_test.txt', 'w') as f:
    start = time.time()
    while time.time() - start < 55:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
            f.write(line + '\n')
        if 'Step test done' in line:
            break

ser.close()
print("Saved to step_test.txt")
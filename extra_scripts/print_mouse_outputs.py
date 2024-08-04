import io
import serial

ser = serial.Serial('/dev/hidraw0', 9600, timeout=5.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

while ser.in_waiting > 0:  # While there are bytes in the input buffer
    try:
        line = sio.readline()
        print(line + '\n')
    except serial.SerialException as e:
        print('Mouse Device error: {}'.format(e))

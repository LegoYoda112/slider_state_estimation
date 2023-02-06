import serial
import re
import numpy as np
from time import sleep
ser = serial.Serial('/dev/ttyACM0',
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1,
                    xonxoff=0,
                    rtscts=0)

# # Restart arduino?
# ser.setDTR(False)
# sleep(1)
# ser.flushInput()
# ser.setDTR(True)

print(ser.name)

# From https://math.stackexchange.com/questions/2975109/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

for i in range(1000):
    line = ser.readline()
    print(str(line))
    rpy = np.array(re.findall(r"[-0-9.]+", str(line)), float)

    if(len(rpy) == 3):
        quat = euler_to_quaternion(rpy)
        print(quat)
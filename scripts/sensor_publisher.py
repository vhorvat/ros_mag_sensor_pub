#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial
import re

def readSerial(device_path):
    ser = serial.Serial(device_path, 115200, timeout=1)
    ser.flush()
    
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    return line
            except Exception as e:
                rospy.logwarn(f"Failed to read from serial port: {e}")

def parseSerialData(line):
    pattern = r"x:\(([-\d\.]+) µT\), y:\(([-\d\.]+) µT\), z:\(([-\d\.]+) µT\), t:\(([-\d\.]+)°C\)" 
    match = re.match(pattern, line)
    
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        z = float(match.group(3))
        t = float(match.group(4))
        return x, y, z, t
    else:
        return None

def sensorPublisher():
    rospy.init_node('sensor_publisher_node', anonymous=True)

    devicePath = rospy.get_param('~device_path', '/dev/ttyACM0')
    rospy.loginfo("Reading data from device: %s", devicePath)

    pub = rospy.Publisher('sensor_data', String, queue_size=10)
    rate = rospy.Rate(100)  

    while not rospy.is_shutdown():
        serialData = readSerial(devicePath)

        if serialData:
            sensorVal = parseSerialData(serialData)
            
            if sensorVal:
                x, y, z, t = sensorVal
                
                sensorDataMsg = "x:{:.2f},y:{:.2f},z:{:.2f},t:{:.2f}".format(x, y, z, t)
                
                rospy.loginfo(sensorDataMsg)
                pub.publish(sensorDataMsg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        sensorPublisher()
    except rospy.ROSInterruptException:
        pass


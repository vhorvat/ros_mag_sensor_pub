#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial
import threading

def readSerial(devicePath):
    ser = serial.Serial(devicePath, 115200, timeout=1)
    ser.flush()
    
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore')
                rospy.loginfo(f"{line}")
                if line:
                    return line
            except Exception as e:
                rospy.logwarn(f"Failed to read from serial: {e}")

def sensorPublisher(sensorName, devicePath):
    pub = rospy.Publisher(f'sensor_data/{sensorName}', String, queue_size=10)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        serialData = readSerial(devicePath)

        if serialData:
            pub.publish(serialData)

        rate.sleep()

def start_sensor_thread(sensorName, devicePath):
    thread = threading.Thread(target=sensorPublisher, args=(sensorName, devicePath))
    thread.start()

if __name__ == '__main__':
    try:

        rospy.init_node('multi_sensor_publisher_node', anonymous=True)
        sensors = [
            {"name": "senzor1", "device_path": "/dev/senzor1"},
            {"name": "senzor2", "device_path": "/dev/senzor2"}
        ]

        for sensor in sensors:
            start_sensor_thread(sensor["name"], sensor["device_path"])

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


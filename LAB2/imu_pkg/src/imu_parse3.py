#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import numpy
from math import sin, pi
from sensor_msgs.msg import Imu, MagneticField
from squaternion import euler2quat


def parse_imu(imu_split):
    '''
    Given pressure (in m fresh) and latitude (in radians) returns ocean depth (in m.).  Uses the formula discovered and presented by Leroy and Parthiot in: Claude C. Leroy and Francois Parthiot, 'Depth-pressure relationships in the oceans and seas', J. Acoustic Society of America, March 1998, p1346-.
    '''

    print('imu split ',imu_split)
    yaw = float(imu_split[1])
    pitch = float(imu_split[2])
    roll = float(imu_split[3])
    MagX = float(imu_split[4])
    MagY = float(imu_split[5])
    MagZ = float(imu_split[6])
    AccelX = float(imu_split[7])
    AccelY = float(imu_split[8])
    AccelZ = float(imu_split[9])
    GyroX = float(imu_split[10])
    GyroY = float(imu_split[11])
    GyroZ = float(imu_split[12])

    imu_msg = Imu()

    imu_quaternion = euler2quat(roll,pitch,yaw)
    print(type(imu_quaternion))

    imu_msg.orientation.x = imu_quaternion.x
    imu_msg.orientation.y = imu_quaternion.y
    imu_msg.orientation.z = imu_quaternion.z
    imu_msg.orientation.w = imu_quaternion.w
    
    imu_msg.angular_velocity.x = GyroX
    imu_msg.angular_velocity.y = GyroY
    imu_msg.angular_velocity.z = GyroZ

    imu_msg.linear_acceleration.x = AccelX
    imu_msg.linear_acceleration.y = AccelY
    imu_msg.linear_acceleration.z = AccelZ

    mag_msg = MagneticField()
    mag_msg.magnetic_field.x = MagX
    mag_msg.magnetic_field.y = MagY
    mag_msg.magnetic_field.z = MagZ

    return imu_msg, mag_msg


if __name__ == '__main__':
    SENSOR_NAME = "IMU"
    rospy.init_node('parse_IMU',anonymous=True)
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',115200)
    sampling_rate = rospy.get_param('~sampling_rate',20)
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.loginfo("Using IMU on port "+serial_port+" at "+str(serial_baud))
    rospy.sleep(0.2)   
    line = port.readline()

    imu_pub = rospy.Publisher(SENSOR_NAME+'/imu', Imu, queue_size=5)
    mag_pub = rospy.Publisher(SENSOR_NAME+'/magnatic_field', MagneticField, queue_size=5)

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing IMU messages.")
    sleep_time = 1.0/sampling_rate
    print(sleep_time)
    ii = 1

    try:
        while not rospy.is_shutdown():
            ii = ii + 1
            line = port.readline()
            print('imu raw data ',line)
            if line == '':
                rospy.logwarn("IMU: No data")
            else:
                if line.startswith('$VNYMR'):
                    try: 
                        imu_data = line.split("*")
                        print('imu no * ',imu_data)
                        imu_split = imu_data[0].split(",")
                        if_null = 0
                        for field in imu_split:
                            if field == '':
                                if_null = 1
                                break
                        
                        if if_null == 1:
                            continue

                        else:  
                            imu_msg, mag_msg = parse_imu(imu_split)
                            imu_pub.publish(imu_msg)
                            mag_pub.publish(mag_msg)
                    except: 
                        rospy.logwarn("Data exception: "+line)
                        continue

            rospy.sleep(sleep_time)
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
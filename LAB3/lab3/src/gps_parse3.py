#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
from math import sin, pi
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from lab3.msg import GPS
import utm


def parse_gps(line_split):
    '''
    Given pressure (in m fresh) and latitude (in radians) returns ocean depth (in m.).  Uses the formula discovered and presented by Leroy and Parthiot in: Claude C. Leroy and Francois Parthiot, 'Depth-pressure relationships in the oceans and seas', J. Acoustic Society of America, March 1998, p1346-.
    '''
  
    gps_message = {"latitude" : 0.0,
                    "longitude" : 0.0,
                    "altitude" : 0.0,
                    "utm_easting" : 0.0,
                    "utm_northing" : 0.0,
                    "fix_quality" : 0.0,
                    "time" : 0.0,
                    "zone" : 0,
                    "letter" : "U"}
    
    gps_message["latitude"] = float(line_split[2]) // 100 + float(line_split[2]) % 100 / 60
    gps_message["longitude"] = float(line_split[4]) // 100 + float(line_split[4]) % 100 / 60

    if line_split[3] == 'S':
        gps_message["latitude"] = -1 * gps_message["latitude"]
    
    if line_split[5] == 'W':
        gps_message["longitude"] = -1 * gps_message["longitude"]
    
    gps_message["altitude"] = float(line_split[9])
    utm_result = utm.from_latlon(gps_message["latitude"], gps_message["longitude"])
    gps_message["utm_easting"] = utm_result[0]
    gps_message["utm_northing"] = utm_result[1]
    gps_message["fix_quality"] = float(line_split[6])
    gps_message["time"] = float(line_split[1])
    gps_message["zone"] = utm_result[2]
    gps_message["letter"] = utm_result[3]

    return gps_message


if __name__ == '__main__':
    SENSOR_NAME = "GPS"
    rospy.init_node('parse',anonymous=True)
    serial_port = rospy.get_param('~port','/dev/ttyACM0')
    serial_baud = rospy.get_param('~baudrate',115200)
    sampling_rate = rospy.get_param('~sampling_rate',20.0)
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.loginfo("Using GPS on port "+serial_port+" at "+str(serial_baud))
    rospy.sleep(0.2)   
    line = port.readline()
    gps_pub = rospy.Publisher(SENSOR_NAME, GPS, queue_size=20)
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing GPS messages.")
    sleep_time = 1/sampling_rate

    try:
        while not rospy.is_shutdown():
            line = port.readline()
            # line = '$GNGGA,123456.12,4124.8963,N,08151.6838,W,5,xx,x.x,280.2,M,x.x,M,x.x,xxxx'
            if line == '':
                rospy.logwarn("GPS: No data")
            else:
                if line.startswith('$GNGGA'):
                    try: 
                        line_split = line.split(",")
                        rospy.loginfo(line_split)
                        # rospy.loginfo(22222)
                        if_null = 0
                        for field in line_split[0:13]:
                            if field == '':
                                if_null = 1
                                break
                        
                        if if_null == 1:
                            continue

                        else:  
                            gps_message = parse_gps(line_split)
                            # rospy.loginfo(11111)
                            gps_pub.publish(latitude=gps_message["latitude"],longitude=gps_message["longitude"],altitude=gps_message["altitude"],utm_easting=gps_message["utm_easting"],utm_northing=gps_message["utm_northing"],zone=gps_message["zone"],letter=gps_message["letter"],fix_quality = gps_message["fix_quality"],time = gps_message["time"])
                            rospy.loginfo(gps_message)
                    except:
                        rospy.logwarn("Data exception: "+line)
                        continue

            # rospy.sleep(sleep_time)
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
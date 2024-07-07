#!/usr/bin/env python

import rospy
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointSetCurrent
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import RCOut
from std_msgs.msg import String
from mavros_msgs.msg import ActuatorControl
from sensor_msgs.msg import NavSatFix
import time

lat = 0
lon = 0

def gps_callback(data):
    global lat, lon
    # data from mavros/global_position/global
    lat = data.latitude
    lon = data.longitude

def getMotorSpeed(x, y):
    if x > 1400 and x < 1600 and (y > 1600 or y < 1400):
        return (y, y)
    elif y > 1400 and y < 1600:
        return (x, 3000 - x)
    else:
        l = int((((y - 1500) * (x - 1500)) / 500))
        r = (y - l)
        return (l, r)

pub = rospy.Publisher('joystick', String, queue_size=10)

def rc_out_callback(data):
    ch1 = data.channels[0]
    ch2 = data.channels[1]
    # ch1_mapped = (ch1 - 1000) * (400 / 1000) + 1300
    # ch2_mapped = (ch2 - 1000) * (400 / 1000) + 1300
    ch1_mapped = ch1
    ch2_mapped = ch2
    st = "[" + str(int(ch1_mapped)) + "," + str(int(ch2_mapped)) + "]"
    pub.publish(st)

    with open("rc_out_log.txt", "a") as log_file:
        log_file.write(f"{time.time()} - {ch1},{ch2} - {ch1_mapped},{ch2_mapped}\n")


def main():
    rospy.init_node('mission_upload_start')
    

    rospy.loginfo("Mission uploaded and started successfully")

    rospy.spin()

if __name__ == '__main__':
    rospy.Subscriber('/mavros/rc/out', RCOut, rc_out_callback)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, gps_callback)
    
    main()

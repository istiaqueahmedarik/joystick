#!/usr/bin/env python

import math
import rospy
from detect_aruco.msg import Num
from aruco_go.msg import wheel
from std_msgs.msg import String, Float64
from mavros_msgs.msg import Imu
import time
from sensor_msgs.msg import LaserScan
import threading


currentHeading = 0


def stage1():
    pass

def stage2():
    # no aruco
    # one aruco
    # alreay seen one aruco
    # two aruco
    
    pass

def stage3():
    pass

def stage4():
    pass
def callback(msg):
    global rotate
    

    global last_msg_time
    last_msg_time = rospy.get_time()
    global entrance
    entrance = True

    # Process the received message
    # Access message fields using msg.field_name
    # rospy.loginfo("Received message: %s", msg)

    # Process the received message here...
    tot = msg.total;
    lst = tot.split(',')
    tuples = []

    for i in range(0, len(lst), 3):
        if(int(lst[i])==269):
            tuples.append((float(lst[i]), float(lst[i+1]), float(lst[i+2])))

    global tups
    tups = tuples

lidar_dist = {}
def lidar_callback(msg):
    global entrance
    global lidar_dist
    for i, distance in enumerate(msg.ranges):
        degree = math.degrees(msg.angle_min + i * msg.angle_increment)
        lidar_dist[round(degree)] = distance
    

def stageControl():
    cur = 1
    while(1):
        if(cur==1):
            stage1()
            cur+=1
        elif(cur==2):
            stage2()
            cur+=1
        elif(cur==3):
            stage3()
            cur+=1
        elif(cur==4):
            stage4()
        else:
            break


def imu_callback(msg):
    global imu
    imu = msg

def headingCallback(msg):
    global currentHeading
    currentHeading = msg.data

def publish_custom_data(data,mainHead,dynamic=False):
    data[0] = max(min(data[0],2000),1000)
    data[1] = max(min(data[1],2000),1000)

    # Create a publisher object
    pub = rospy.Publisher('joystick', String, queue_size=10)

    # Create a custom message
    custom_msg = wheel()
    custom_msg.left = data[0]
    custom_msg.right = data[1]
    wh = "[" + str(data[0]) + "," + str(data[1]) + "]"
    # Publish the custom message
    pub.publish(wh)
    if(dynamic==False): 
        return
    time.sleep(0.5) # wait for rotation
    changedHeading = currentHeading
    if(abs(changedHeading-mainHead)<3):
        factor = 10
        publish_custom_data((data[0]+factor,data[1]+factor),dynamic)


def listener():
    # Initialize the ROS node
    rospy.init_node('aruco_go', anonymous=True)

    # Create a subscriber object
    rospy.Subscriber("aruco_info", Num, callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
    rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, headingCallback)

    
    # Create a timer to publish default values if no message is received
    # rospy.Timer(rospy.Duration(0.01), no_msg_timer)  # Adjust the timer period as needed
    t1 = threading.Thread(target=stageControl)
    t1.start()
    # Spin and process incoming messages
    rospy.spin()

if __name__ == '__main__':
    listener()
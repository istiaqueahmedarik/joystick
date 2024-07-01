#!/usr/bin/env python

import rospy
from detect_aruco.msg import Num
from aruco_go.msg import wheel
from std_msgs.msg import String
import time
def map_(x):
    return (abs(x) * 200) / 350
completed = dict()
# Callback function to process incoming messages
last_msg_time = None

def callback(msg):
    global last_msg_time
    last_msg_time = rospy.get_time()

    # Process the received message
    # Access message fields using msg.field_name
    # rospy.loginfo("Received message: %s", msg)

    # Process the received message here...
    tot = msg.total;
    lst = tot.split(',')
    tuples = []

    for i in range(0, len(lst), 3):
        tuples.append((float(lst[i]), float(lst[i+1]), float(lst[i+2])))

    sorted_tuples = sorted(tuples, key=lambda x: x[2])

    print(sorted_tuples)

    mission = 0
    for i in range(len(sorted_tuples)):
        dis = float(sorted_tuples[i][2])
        x = float(sorted_tuples[i][1])
        id = int(sorted_tuples[i][0])

        if(dis<0.45 and completed[id]>=1500):
            continue
        elif(dis<0.45):
            try:
                completed[id] +=1
            except KeyError:
                completed[id] = 0
            break
        else:
            mission +=1
            left = 1500
            right = 1500

            if x > 350:
                left = 1700
                right = 1400
            elif x < -350:
                left = 1400
                right = 1500
            elif x > 50:
                right = int(1700 - map_(x))
                left = int(1500 + map_(x))
            
                
            elif x < -50:
                left = int(1700 - map_(x))
                right = int(1500 + map_(x))
               
            else:
                right = 1700
                left = 1700

            # After processing, publish custom data
            publish_custom_data((left, right))
            break
    if(mission==0):
        publish_custom_data((1500, 1500)) 

def publish_custom_data(data):
    # Create a publisher object
    pub = rospy.Publisher('joystick', String, queue_size=10)

    # Create a custom message
    custom_msg = wheel()
    custom_msg.left = data[0]
    custom_msg.right = data[1]
    wh = "[" + str(data[0]) + "," + str(data[1]) + "]"

    # Publish the custom message
    pub.publish(wh)

def no_msg_timer(event):
    global last_msg_time
    if last_msg_time is None or rospy.get_time() - last_msg_time > 1.0:  # Adjust the time threshold as needed
        publish_custom_data((1500, 1500))

def listener():
    # Initialize the ROS node
    rospy.init_node('aruco_go', anonymous=True)

    # Create a subscriber object
    rospy.Subscriber("aruco_info", Num, callback)

    # Create a timer to publish default values if no message is received
    rospy.Timer(rospy.Duration(0.01), no_msg_timer)  # Adjust the timer period as needed

    # Spin and process incoming messages
    rospy.spin()

if __name__ == '__main__':
    listener()
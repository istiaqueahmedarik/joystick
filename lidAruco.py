#!/usr/bin/env python

import math
import rospy
from detect_aruco.msg import Num
from aruco_go.msg import wheel
from std_msgs.msg import String
import time
from sensor_msgs.msg import LaserScan

def map_(x):
    return (abs(x) * 200) / 350

entrance = False
rotate = False
findOnce = False
# Callback function to process incoming messages
last_msg_time = None
tups = False
OBSTACLE_AVOIDANCE_FLAG = False
def aruco_path_for_one(tuple):
    global findOnce
    if(findOnce):
        publish_custom_data((1500,1500))
        return
    global rotate
    dis = float(tuple[2])
    x = float(tuple[1])
    id = int(tuple[0])
    if(abs(dis-9.0)<0.1):
        
        publish_custom_data((1200,1800));
      


        time.sleep(1)
        return
    print(dis, x, id)
    if(dis<0.45):
        return publish_custom_data((1500, 1500))
    
    else:
        left = 1500
        right = 1500

        if x > 350:
            left = 1700
            right = 1400
        elif x < -350:
            left = 1400
            right = 1700
        elif x > 50:
            right = int(1700 - map_(x))
            left = int(1500 + map_(x))
        
            
        elif x < -50:
            left = int(1700 - map_(x))
            right = int(1500 + map_(x))
            
        else:
            right = 1700
            left = 1700
        rospy.loginfo("ekta aruco er dike agaitese")
        # After processing, publish custom data
        publish_custom_data((left, right))

def aruco_path_for_two(tuple1, tuple2):
    global findOnce
    rospy.loginfo("2 ta aruco i paise")
    dis1 = float(tuple1[2])
    x1 = float(tuple1[1])
    id1 = int(tuple1[0])

    dis2 = float(tuple2[2])
    x2 = float(tuple2[1])
    id2 = int(tuple2[0])
    
    if(min(dis1,dis2)>=8.0):
        rospy.loginfo("8 er beshi distance so ektar dike agaitese")
        if(findOnce):
            rospy.loginfo("2 ta aruco ekbar paise so theme gese")
            publish_custom_data((1500,1500))
            return
        if(dis1>dis2):
            rospy.loginfo("beshi dist jetar shedike jaitese")
            aruco_path_for_one(tuple1)
        else:
            aruco_path_for_one(tuple2)

        return

    x = (x1 + x2)/2
    global OBSTACLE_AVOIDANCE_FLAG
    if(dis1<=3.5 or dis2<=3.5):
        rospy.loginfo("6 er kom distance 2 ta aruco er jekono ekta so theme jabe")
        # needs to align
        # print(x)
        OBSTACLE_AVOIDANCE_FLAG=True
        obstacleAvoidance()
        #publish_custom_data((1500,1500))
        return
    
    print(x1, x2, x,dis1,dis2)
    findOnce = True
    rospy.loginfo("2 ta aruco er majhe jaitese")
    left = 1500
    right = 1500
    if x > 100:
        left = 1600
        right = 1400
    elif x < -100:
        left = 1400
        right = 1600
    elif x > 30:
        right = int(1700 - map_(x))
        left = int(1500 + map_(x))
    
        
    elif x < -30:
        left = int(1700 - map_(x))
        right = int(1500 + map_(x))
        
    else:
        right = 1700
        left = 1700

    # After processing, publish custom data
    publish_custom_data((left, right))

def obstacleAvoidance():
    OBSTACLE_AVOIDANCE_FLAG=True
    rospy.loginfo("obstacle avoid mode")
    neutral_speed = 1500
    forward_speed = 1750  
    backward_speed = 1250  
    front_distance = lidar_dist[0]
    left_distance =  lidar_dist[90]
    left_corner_distance = lidar_dist[45]
    for i in range(45):
        left_corner_distance = min(left_corner_distance,lidar_dist[i])
    right_distance = lidar_dist[-90]
    right_corner_distance = lidar_dist[-45]

    for i in range(45):
        right_corner_distance = min(right_corner_distance,lidar_dist[-1*i])
    
    if(left_distance == float('inf')):
        left_distance = 0.1
    if(right_distance == float('inf')):
        right_distance = 0.1

    l_speed = neutral_speed
    r_speed = neutral_speed
    if(lidar_dist[0]<1):
        if(left_distance>right_distance):
            publish_custom_data((1800,1100))
            return
        else:
            publish_custom_data((1100,1800))
            return
    if  left_corner_distance < 1.3 and right_corner_distance >1.3:
        l_speed = 1800
        r_speed = 1100
    elif  right_corner_distance < 1 and left_corner_distance >1:
        l_speed = 1100
        r_speed = 1800
    elif( left_corner_distance>1.8 and right_corner_distance>1.8):
        if left_corner_distance < right_corner_distance:
            speed_diff = int((right_corner_distance - left_corner_distance) * 500)  
            l_speed = forward_speed
            r_speed = max(neutral_speed, forward_speed - speed_diff)
        else:
            speed_diff = int((left_corner_distance - right_corner_distance) * 500)  
            l_speed = max(neutral_speed, forward_speed - speed_diff)
            r_speed = forward_speed
        
    elif left_distance < right_distance:
        speed_diff = int((right_distance - left_distance) * 500)  
        l_speed = forward_speed
        r_speed = max(neutral_speed, forward_speed - speed_diff)
    else:
        speed_diff = int((left_distance - right_distance) * 500)  
        l_speed = max(neutral_speed, forward_speed - speed_diff)
        r_speed = forward_speed
    
    publish_custom_data((l_speed,r_speed))

lidar_dist = {}
def lidar_callback(msg):
    global entrance
    global lidar_dist
    for i, distance in enumerate(msg.ranges):
        degree = math.degrees(msg.angle_min + i * msg.angle_increment)
        lidar_dist[round(degree)] = distance
    if(OBSTACLE_AVOIDANCE_FLAG):
        obstacleAvoidance()
        return
    if(entrance==False):
        rospy.loginfo("Kono aruco pay nai so ghurtese")
        publish_custom_data((1200,1800))
        entrance = True
        global rotate
        rotate = True
        time.sleep(1)
    return
    
    

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

    # sorted_tuples = sorted(tuples, key=lambda x: x[2])

    # print(sorted_tuples)
    # print(len(tuples))
    global OBSTACLE_AVOIDANCE_FLAG
    if(OBSTACLE_AVOIDANCE_FLAG):
        obstacleAvoidance()
        return
    if(len(tuples)>=2):
        aruco_path_for_two(tuples[0], tuples[1])
    elif(len(tuples)):
        aruco_path_for_one(tuples[0])

    
 

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
    if(OBSTACLE_AVOIDANCE_FLAG):
        return
    global last_msg_time
    global entrance
    if last_msg_time is None or rospy.get_time() - last_msg_time > 1.5:  # Adjust the time threshold as needed
        rospy.loginfo("kono zed feedback nai so rotate start")
        entrance = False

def listener():
    # Initialize the ROS node
    rospy.init_node('aruco_go', anonymous=True)

    # Create a subscriber object
    rospy.Subscriber("aruco_info", Num, callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    
    # Create a timer to publish default values if no message is received
    rospy.Timer(rospy.Duration(0.01), no_msg_timer)  # Adjust the timer period as needed

    # Spin and process incoming messages
    rospy.spin()

if __name__ == '__main__':
    listener()
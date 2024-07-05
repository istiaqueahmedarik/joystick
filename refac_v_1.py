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
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointSetCurrent
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import RCOut
from std_msgs.msg import String
from mavros_msgs.msg import ActuatorControl
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointReached

from std_msgs.msg import String
import time
from sensor_msgs.msg import LaserScan

def map_(x):
    return (abs(x) * 200) / 350
global_waypoints = []
flag=False

currentHeading = 0
currentLang, currentLat = 0, 0
landingLang, landingLat = 0, 0
stage2_complete = threading.Event()
stage2_complete1 = threading.Event()

def gps_callback(msg):
    global currentLang, currentLat
    currentLang = msg.longitude
    currentLat = msg.latitude

def getMotorSpeed(x,y):
    if(x>1400 and x<1600 and (y>1600 or y<1400)):
        return (y,y)
    elif(y>1400 and y<1600):
        return (x,3000-x)
    else:
        l = int((((y-1500)*(x-1500))/500))
        r = (y-l)
        return (l,r)

pub = rospy.Publisher('joystick', String, queue_size=10)

def rc_out_callback(data):
    ch1 = data.channels[0]
    ch2 = data.channels[1]

    st = "[" + str(int(ch2)) + "," + str(int(ch1)) + "]"
    pub.publish(st)

def upload_mission(waypoints):
    rospy.wait_for_service('/mavros/mission/push')
    try:
        waypoint_push = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        resp = waypoint_push(start_index=0, waypoints=waypoints)
        rospy.loginfo(f"Waypoints uploaded: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def clear_mission():
    rospy.wait_for_service('/mavros/mission/clear')
    try:
        waypoint_clear = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
        resp = waypoint_clear()
        rospy.loginfo(f"Waypoints cleared: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def set_current_waypoint(seq):
    rospy.wait_for_service('/mavros/mission/set_current')
    try:
        waypoint_set_current = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)
        resp = waypoint_set_current(wp_seq=seq)
        rospy.loginfo(f"Current waypoint set: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def set_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        resp = set_mode_srv(custom_mode=mode)
        rospy.loginfo(f"Mode set to {mode}: {resp.mode_sent}")
        return resp.mode_sent
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def disarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        resp = arming_srv(value=False)
        rospy.loginfo(f"Disarmed: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def arm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        resp = arming_srv(value=True)
        rospy.loginfo(f"Armed: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def create_waypoint(lat, lon, alt):
    wp = Waypoint()
    wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp.command = 16  # MAV_CMD_NAV_WAYPOINT
    wp.is_current = False
    wp.autocontinue = True
    wp.param1 = 0  # Hold time in seconds
    wp.param2 = 0  # Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)
    wp.param3 = 0  # 0 to pass through WP
    wp.param4 = float('nan')  # Desired yaw angle at waypoint (NaN for unchanged)
    wp.x_lat = lat
    wp.y_long = lon
    wp.z_alt = alt
    return wp

def safe():
    disarm()
    set_mode("MANUAL")
    rospy.loginfo("Mission upload aborted")
    rospy.signal_shutdown("Mission upload aborted")
    pub.publish("[1500,1500]")

def waypoint_reached_callback(msg):
    global flag
    rospy.loginfo("Waypoint %d reached", msg.wp_seq)
    if msg.wp_seq == len(global_waypoints) - 1:
        if(flag):
            stage2_complete1.set()  # Signal that the mission is complete
        else:
            stage2_complete.set()  # Signal that the mission is complete
        flag = True

def stage1():
    global landingLat
    global landingLang
    landingLang = currentLang
    landingLat = currentLat
    pass

def gps_move(gpslang, gpslat):
    global global_waypoints

    waypoints = [create_waypoint(landingLang, landingLat, 10), create_waypoint(gpslang, gpslat, 10)]
    global_waypoints = waypoints

    disarm()

    # Clear any existing mission
    if not clear_mission():
        return

    # Upload the new mission
    if not upload_mission(waypoints):
        return

    # Set the first waypoint as current
    if not set_current_waypoint(0):
        return

    if not set_mode("GUIDED"):
        return

    # Set mode to AUTO
    if not set_mode("AUTO"):
        return

    # Arm the vehicle
    if not arm():
        return

def rotate():
    publish_custom_data([1800, 1200], 1)

findonce = False
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
        # obstacleAvoidance()
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
    elif x > 60:
        right = int(1700 - map_(x))
        left = int(1500 + map_(x))
    
        
    elif x < -60:
        left = int(1700 - map_(x))
        right = int(1500 + map_(x))
        
    else:
        right = 1700
        left = 1700

    # After processing, publish custom data
    publish_custom_data((left, right))



armove_stage = threading.Event()
def ArMove():
    while(True):
        if(OBSTACLE_AVOIDANCE_FLAG):
            rospy.loginfo("Obstacle avoidance er jonno return")
            break
        if(len(tups)==0):
            rospy.loginfo("Kono aruco pay nai so ghurtese")
            publish_custom_data([1200, 1800], 1)
            time.sleep(1)
            continue
        if(len(tups)>=2):
            aruco_path_for_two(tups[0], tups[1])
        elif(len(tups)):
            aruco_path_for_one(tups[0])

def stage2():
    
    gpslang, gpslat = map(float, input(f"Enter GPS coordinates for mission 1: ").split())
    gps_move(gpslang, gpslat)

    rospy.loginfo(f"Stage 2 mission 1 started. Waiting for completion...")
    stage2_complete.wait()  
    stage2_complete.clear() 

    rospy.loginfo(f"Stage 2 mission 1 completed.")

    gpslang, gpslat = map(float, input(f"Enter GPS coordinates for mission 2: ").split())
    gps_move(gpslang, gpslat)

    rospy.loginfo(f"Stage 2 mission 2 started. Waiting for completion...")

    stage2_complete1.wait()
    stage2_complete1.clear()

    rospy.loginfo(f"Stage 2 mission 2 completed.")

    # now we will move to aruco two thing can happen, we didn't find any aruco or we found aruco

    ArMove()

    
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
  
def detectTunnel():
    #this function will determine if vehicle is inside a tunnel or not
    #if vehicle is inside a tunnel, it will return True otherwise False
    #tunnel width is 2.25-2.5m
    #first we will check if any distance is similar to tunnel width at least once
    #after that if dissimilarity happen we will check if surrounding distance is >3m
    #if surrounding distance is >3m then we will consider it as outside
    #if surrounding distance is <3m then we will consider it as inside
    
    tunnel_width = 2.5
    tunnel_width_lower = 2.25


    #we will check summation of opposite degree distance
    getInside = False

    while(True):
        flag = False
        for i in range(90):
            d1 = lidar_dist[i]
            d2 = lidar_dist[180-i]
            sm = d1+d2
            if(abs(sm-tunnel_width)<0.3 or abs(sm-tunnel_width_lower)<0.3):
                getInside = True
                flag = True
                break
        if(flag==False and getInside==True):
            for i in range(90):
                if(lidar_dist[i]>=3 or lidar_dist[180-i]>=3):
                    return True
            
        obstacleAvoidance()
        

def stage3():
    detectTunnel()

def stage4():
    pass

def callback(msg):
    global rotate
    global last_msg_time
    last_msg_time = rospy.get_time()
    global entrance
    entrance = True

    tot = msg.total
    lst = tot.split(',')
    tuples = []

    for i in range(0, len(lst), 3):
        if int(lst[i]) == 269:
            tuples.append((float(lst[i]), float(lst[i + 1]), float(lst[i + 2])))
            if(len(tuples)==2):
                mn = min(tuples[0][2], tuples[1][2])
                if(mn<=11):
                    stage2_complete1.set()
            else:
                if(tuples[0][2]<=11):
                    stage2_complete1.set()

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
            # entrance
            stage1()
            cur += 1
        elif(cur==2):
            # move to two gps location
            stage2()
            cur += 1
        elif(cur==3):
            stage3()
            cur += 1
        elif(cur==4):
            stage4()
            cur += 1
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


    custom_msg = wheel()
    custom_msg.left = data[0]
    custom_msg.right = data[1]
    wh = "[" + str(data[0]) + "," + str(data[1]) + "]"
    pub.publish(wh)
    if dynamic == False:
        return
    time.sleep(0.5) # wait for rotation
    changedHeading = currentHeading
    if abs(changedHeading - mainHead) < 10:
        factor = 10
        loc = data[0]
        loc1 = data[1]
        if data[0] > 1500:
            loc += factor
        if data[1] > 1500:
            loc1 += factor
        if data[0] < 1500:
            loc -= factor
        if data[1] < 1500:
            loc1 -= factor
        publish_custom_data([loc, loc1], mainHead, True)

def listener():
    rospy.init_node('aruco_go', anonymous=True)

    rospy.Subscriber("aruco_info", Num, callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
    rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, headingCallback)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, gps_callback)
    rospy.Subscriber("/mavros/mission/reached", WaypointReached, waypoint_reached_callback)


    t1 = threading.Thread(target=stageControl)
    t1.start()

    rospy.spin()

if __name__ == '__main__':
    listener()

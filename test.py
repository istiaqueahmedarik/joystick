#!/usr/bin/env python

import rospy
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointSetCurrent
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import RCOut
from std_msgs.msg import String
from mavros_msgs.msg import ActuatorControl
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
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.channels)
    ch1 = data.channels[0]
    ch2 = data.channels[1]
    # map ch1 andch2 from 1000 to 2000 to 1300 to 1700
    ch1 = (ch1-1000)*(400/1000)+1300
    ch2 = (ch2-1000)*(400/1000)+1300

    
    
    st = "[" + str(int(ch1)) + "," + str(int(ch1)) + "]"
    # rospy.loginfo(st)
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
def main():
    rospy.init_node('mission_upload_start')
    rospy.on_shutdown(safe)
    # Create a list of waypoints
    waypoints = [
        create_waypoint(23.8377406, 90.3575542, 10), 
        create_waypoint(23.8377406, 90.3575542, 10),  # Example coordinates
          # Example coordinates
        
    ]

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

    rospy.loginfo("Mission uploaded and started successfully")

    rospy.spin()

if __name__ == '__main__':
    rospy.Subscriber('/mavros/rc/out', RCOut, rc_out_callback)
    
    main()
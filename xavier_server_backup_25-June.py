import math
import rospy
from std_msgs.msg import Float64MultiArray, String
from flask_cors import CORS
from flask import Flask, jsonify, request
from flask_socketio import SocketIO, emit
import time
import threading
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt32,String,Float64
from mavros_msgs.msg import Waypoint,ParamValue,RCOut,WaypointReached
from mavros_msgs.srv import WaypointPush, WaypointPushRequest, WaypointPull, WaypointPullRequest,CommandBool,SetMode,WaypointClear,ParamSet
import signal
import sys
import zlib
def compress_string(string):
    compressed_data = zlib.compress(string.encode())
    return compressed_data

def decompress_string(compressed_data):
    decompressed_string = zlib.decompress(compressed_data).decode()
    return decompressed_string
def signal_handler(sig, frame):
    print('Server is shutting down...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
app = Flask(__name__)




socketio = SocketIO(app, allow_upgrades=True, cors_allowed_origins='*')
CORS(app, resources={r"/*": {"origins": "*", "allow_headers": ["Content-Type", "Another-Allowed-Header"]}})
rospy.init_node('server', anonymous=True)

push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
pull_srv = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
waypoints = [] 

pub = rospy.Publisher('joystick', String, queue_size=20)
rospy.Rate(10)
mode = rospy.Publisher('mode', String, queue_size=20)
current_mode = "manual"
@socketio.on('connect')
def connect():
    print('Client connected')
    rospy.loginfo('A new Client connected')
@socketio.on('zavier')
def zavier(data):
    print(data)
armPub = rospy.Publisher('joyArm', String, queue_size=20)
@socketio.on('joystick_data')
def joystick_data(data):
    if(current_mode!='manual'):
        return
    
    rospy.loginfo("Joystick data recieved")
    pub.publish(decompress_string(data))
    rospy.sleep(0.1)
@socketio.on('armMsg')
def armMsgData(data):
    armPub.publish(data);
    rospy.sleep(0.1)


def main():
    # rospy.wait_for_service('mavros/set_mode')
    # set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
    # response = set_mode_service(custom_mode='MANUAL')
    # print(response)
    # rospy.wait_for_service('mavros/cmd/arming')
    # arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    # response = arm_service(False)
    # print(response)
    # rospy.wait_for_service('mavros/mission/clear')
    # clear_service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
    # response = clear_service()
    # print(response)
    socketio.run(app, host='0.0.0.0', port=5476,debug=True,use_reloader=True,log_output=True)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    

    # if current mode is manual only then joystick will work
    # if current mode is auto only pixhawk will work
    # if current mode is rotate then only rotate will work
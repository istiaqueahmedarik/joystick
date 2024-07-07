
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
from threading import Timer
import time

# Global variable to track the last time data was received
last_data_time = time.time()
def signal_handler(sig, frame):
    print('Server is shutting down...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
app = Flask(__name__)

from engineio.payload import Payload
Payload.max_decode_packets=5000000000000000000000000000000

flag=False


socketio = SocketIO(app, allow_upgrades=True, cors_allowed_origins='*')
CORS(app, resources={r"/*": {"origins": "*", "allow_headers": ["Content-Type", "Another-Allowed-Header"]}})
rospy.init_node('server', anonymous=True)

def check_joystick_data():
    if(not flag):
        return
    global last_data_time
    if time.time() - last_data_time > 3:
        pub.publish('[1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]')
        Timer(1, check_joystick_data).start()

    Timer(1, check_joystick_data).start()
pub = rospy.Publisher('joystick', String, queue_size=10)
rospy.Rate(250)
mode = rospy.Publisher('mode', String, queue_size=10)
current_mode = "manual"
@socketio.on('connect')
def connect():
    global flag
    flag=True
    print('Client connected')
    rospy.loginfo('A new Client connected')
@socketio.on('disconnect')
def disconnect():
    global flag
    flag=False
    print('Client disconnected')
    pub.publish('[1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]')
    rospy.loginfo('Client disconnected')
@socketio.on('zavier')
def zavier(data):
    print(data)
armPub = rospy.Publisher('joyArm', String, queue_size=10)
@socketio.on('joystick_data')
def joystick_data(data):
    global last_data_time
    if(current_mode!='manual'):
        return
    
    last_data_time = time.time()
    
    rospy.loginfo(f"Joystick data recieved {data}")
    pub.publish(data)
    
Timer(1, check_joystick_data).start()

def main():

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
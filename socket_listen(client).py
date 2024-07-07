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
from std_msgs.msg import UInt32, String, Float64
from mavros_msgs.msg import Waypoint, ParamValue, RCOut, WaypointReached
from mavros_msgs.srv import WaypointPush, WaypointPushRequest, WaypointPull, WaypointPullRequest, CommandBool, SetMode, WaypointClear, ParamSet
import signal
import sys
import zlib

def signal_handler(sig, frame):
    print('Server is shutting down...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
app = Flask(__name__)

from engineio.payload import Payload
Payload.max_decode_packets = 5000000000000000000000000

socketio = SocketIO(app, allow_upgrades=True, cors_allowed_origins='*')
CORS(app, resources={r"/*": {"origins": "*", "allow_headers": ["Content-Type", "Another-Allowed-Header"]}})
rospy.init_node('server', anonymous=True)

pub = rospy.Publisher('joystick', String, queue_size=10)
mode = rospy.Publisher('mode', String, queue_size=10)
armPub = rospy.Publisher('joyArm', String, queue_size=10)
current_mode = "manual"

# disconnect_timer = None
# timeout = 5  # 3 seconds timeout

# def reset_disconnect_timer():
#     global disconnect_timer
#     if disconnect_timer:
#         disconnect_timer.cancel()
#     disconnect_timer = threading.Timer(timeout, send_default_data)
#     disconnect_timer.start()

# def send_default_data():
#     default_data = '[1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]'
#     pub.publish(default_data)
#     rospy.loginfo('No data received for 3 seconds. Sending default data.')
#     print('No data received for 3 seconds. Sending default data.')

@socketio.on('connect')
def connect():
    print('Client connected')
    rospy.loginfo('A new Client connected')

@socketio.on('disconnect')
def disconnect():
    print('Client disconnected')
    pub.publish('[1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]')
    rospy.loginfo('Client disconnected')

@socketio.on('zavier')
def zavier(data):
    print(data)

@socketio.on('joystick_data')
def joystick_data(data):
    if current_mode != 'manual':
        return

    rospy.loginfo(f"Joystick data received: {data}")
    pub.publish(data)
    # reset_disconnect_timer()

def main():
    socketio.run(app, host='0.0.0.0', port=5476, debug=True, use_reloader=True, log_output=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

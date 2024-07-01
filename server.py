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

def signal_handler(sig, frame):
    print('Server is shutting down...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
app = Flask(__name__)

from engineio.payload import Payload
Payload.max_decode_packets = 500


socketio = SocketIO(app, allow_upgrades=True, cors_allowed_origins='*')
CORS(app, resources={r"/*": {"origins": "*", "allow_headers": ["Content-Type", "Another-Allowed-Header"]}})
rospy.init_node('server', anonymous=True)

push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
pull_srv = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
waypoints = [] 

pub = rospy.Publisher('joystick', String, queue_size=1)
rospy.Rate(10)
mode = rospy.Publisher('mode', String, queue_size=1)
current_mode = "manual"
@socketio.on('connect')
def connect():
    print('Client connected')
    rospy.loginfo('A new Client connected')
@socketio.on('zavier')
def zavier(data):
    print(data)
armPub = rospy.Publisher('joyArm', String, queue_size=1)
@socketio.on('joystick_data')
def joystick_data(data):
    if(current_mode!='manual'):
        return
    

    socketio.emit('rover',{
        'message': 'Joystick data received',
    })
    pub.publish(data)
    rospy.sleep(0.1)
@socketio.on('armMsg')
def armMsgData(data):
    armPub.publish(data);
    rospy.sleep(0.1)

def get_new_coordinates(lng,lat,dist,angle,curHeading):
    # dist is in meters
    # angle is in degrees
    # curHeading is in degrees
    R = 6371e3 
    lat1 = math.radians(lat)
    lng1 = math.radians(lng)
    angle = math.radians(angle)
    curHeading = math.radians(curHeading)
    d = dist/R
    lat2 = math.asin(math.sin(lat1)*math.cos(d) + math.cos(lat1)*math.sin(d)*math.cos(angle))
    lng2 = lng1 + math.atan2(math.sin(angle)*math.sin(d)*math.cos(lat1), math.cos(d)-math.sin(lat1)*math.sin(lat2))
    lat2 = math.degrees(lat2)
    lng2 = math.degrees(lng2)
    return lat2,lng2





@app.route('/stop', methods=['POST'])
def stop():
    if(current_mode!='auto'):
        return jsonify({'error': 'Vehicle must be in auto mode to stop'}), 400
    socketio.emit('rover',{
        'message': 'Vehicle stopped',
    })
    
    rospy.wait_for_service('mavros/set_mode')
    set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
    response = set_mode_service(custom_mode='MANUAL')
    print(response)
    rospy.wait_for_service('mavros/cmd/arming')
    arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    response = arm_service(False)
    print(response)
    if response.success:
        return jsonify({'message': 'Vehicle stopped'}), 200
    else:
        return jsonify({'message': 'Failed to stop vehicle'}), 400

def global_position_callback(data):
    global globalLatitude
    global globalLongitude
    latitude = data.latitude
    longitude = data.longitude
    globalLatitude = latitude
    globalLongitude = longitude
    
    # rospy.loginfo("Latitude: %f, Longitude: %f", latitude, longitude)
    socketio.emit('global_position', {'data': {
        'latitude': latitude,
        'longitude': longitude
    }})

def compass_heading_callback(msg):
    global compass_heading
    compass_heading = msg.data
    socketio.emit('compass_heading', {'data': msg.data})
def ros_callback_waypoint(msg):
    if(current_mode!='auto'):
        return
    print(str(msg))
    # socketio.emit('waypoint_message', {'data': msg})
    waypoints.append(msg)

def rc_out_callback(msg):
    if(current_mode!='auto'):
        return
    socketio.emit('rover',{
        'message': 'Px4 is working!',
    })
    channels = msg.channels
    servo1 = channels[0]
    servo2 = channels[1]
    left_motor = 1500 + (servo1-1500)
    right_motor = 1500 + (servo2-1500)
    if(left_motor>1500):
        diff = left_motor-1500
        left_motor = 1500-diff
    else:
        diff = 1500-left_motor
        left_motor = 1500+diff
    if(right_motor>1500):
        diff = right_motor-1500
        right_motor = 1500-diff
    else:
        diff = 1500-right_motor
        right_motor = 1500+diff
    
    left_motor = int(4*(left_motor-1000)/10+1300)
    right_motor = int(4*(right_motor-1000)/10+1300)


    
    
    dt = "["+str(left_motor)+","+str(right_motor)+"]"
    print(dt)
    pub.publish(dt)
def publish_data():
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        mode.publish(current_mode)
        rate.sleep()

def go_to(lat,lon):
    print(lat,lon)
    global current_mode
    current_mode = 'auto'
    rospy.wait_for_service('mavros/set_mode')
    set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
    response = set_mode_service(custom_mode='MANUAL')
    print(response)
    rospy.wait_for_service('mavros/cmd/arming')
    arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    response = arm_service(False)
    print(response)
    rospy.wait_for_service('mavros/mission/clear')
    clear_service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
    response = clear_service()
    print(response)
    waypoints.clear()
    socketio.emit('rover',{
        'message': 'Cleared previous waypoints',
    })


    waypoint = Waypoint()
    waypoint.frame = 3 # MAV_FRAME_GLOBAL_RELATIVE_ALT means that the altitude is relative to the home position 
    waypoint.command = 16  # MAV_CMD_NAV_WAYPOINT is the command to navigate to a waypoint

    waypoint.is_current = False
    waypoint.autocontinue = True
    waypoint.param1 = 5
    waypoint.param2 = 0
    waypoint.param3 = 0
    waypoint.param4 = 0
    waypoint.x_lat = float(globalLatitude)
    waypoint.y_long = float(globalLongitude)
    waypoint.z_alt = float(0)
    waypoints.append(waypoint)
    push_request = WaypointPushRequest()
    push_request.waypoints.extend(waypoints)
    res = push_srv(push_request)
    print(res)
    socketio.emit('rover',{
        'message': 'Current position set as waypoint',
    })


    waypoint = Waypoint()
    waypoint.frame = 3 # MAV_FRAME_GLOBAL_RELATIVE_ALT means that the altitude is relative to the home position 
    waypoint.command = 16  # MAV_CMD_NAV_WAYPOINT is the command to navigate to a waypoint

    waypoint.is_current = False
    waypoint.autocontinue = True
    waypoint.param1 = 5
    waypoint.param2 = 0
    waypoint.param3 = 0
    waypoint.param4 = 0
    waypoint.x_lat = float(lat)
    waypoint.y_long = float(lon)
    waypoint.z_alt = float(0)
    waypoints.append(waypoint)
    push_request = WaypointPushRequest()
    push_request.waypoints.extend(waypoints)
    res = push_srv(push_request)
    print(res)
    socketio.emit('rover',{
        'message': 'Destination set as waypoint',
    })

    rospy.wait_for_service('mavros/set_mode')
    set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
    response = set_mode_service(custom_mode='MANUAL')
    print(response)
    rospy.wait_for_service('mavros/cmd/arming')
    arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    response = arm_service(True)
    print(response)
    rospy.wait_for_service('mavros/set_mode')
    set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
    response = set_mode_service(custom_mode='AUTO')
    print(response)

    socketio.emit('rover',{
        'message': 'Vehicle is moving to destination',
    })



# when detected
def arCode(msg):
    lst = msg.data.split(',')
    if((int(lst[0])==1500 and int(lst[1])==1500) or (current_mode!='rotate' and current_mode!='arcode')):
        return
    if((int(lst[0])==1500 and int(lst[1])==1500) and current_mode=='arcode'):
        socketio.emit('rover',{
            'message': 'ArCode Detection Complete and rover is stopped and should going back...',
        })
        decreaseIndex()
        if(hasAnyVal()):
            newMission()
        
    left = int(lst[0])
    right = int(lst[1])
   
    current_mode = "arcode"
    pub.publish("["+str(left)+","+str(right)+"]")
    socketio.emit('rover',{
        'message': 'ArCode detected and rover is moving',
    })
    

def newMission():
    socketio.emit('rover',{
        'message': 'New Mission Started',
    })
    lst = []
    with open('plan.txt', 'r') as f:
        data = f.read()
        lst = eval(data)
    curIdx = 0
    with open('currentIdx.txt', 'r') as f:
        curIdx = int(f.read())
    socketio.emit('rover',{
        'message': 'Current Index: '+str(curIdx),
    })
    data = lst[curIdx]
    if(data['mode']=='mission'):
        go_to(data['lat'],data['long'])
    elif(data['mode']=='rotate'):
        rotate(data['sec'])

def mission_compelete(msg):
    print("Mission Compelete")
    socketio.emit('rover',{
        'message': 'Mission Compelete',
    })
    increaseIndex()
    if(hasAnyVal()):
        newMission()

def rotate(sec):
    current_mode = "rotate"
    socketio.emit('rover',{
        'message': 'Rotating for '+str(sec)+' seconds',
    })
    st = time.time()
    while(time.time()-st<sec):
        if(current_mode!='rotate'):
            break
        pub.publish("[1700,1300]")
    pub.publish("[1500,1500]")
    if(current_mode!='arcode'):
        increaseIndex()
        if(hasAnyVal()):
            newMission()
    

# curl -X POST -H "Content-Type: application/json" -d '{"distance":10,"sec":3}' http://192.168.43.17:5476/octaAndRotate
@app.route('/octaAndRotate', methods=['POST'])
def octaAndRotate():
    socketio.emit('rover',{
        'message': 'Octa and Rotate',
    })
    data = request.get_json()
    dist = data['distance']
    sec = data['sec']
    lst =  [0,45,90,135,180,225,270,315,360,45]
    arr = []
    with open('plan.txt', 'r') as f:
        data = f.read()
        arr = eval(data)

    for bearing in lst:
        print(bearing)
        new_latitude, new_longitude = get_new_coordinates(globalLongitude, globalLatitude , dist, bearing,compass_heading)
        arr.append({"mode":"mission","long":new_longitude,"lat":new_latitude})
        arr.append({"mode":"rotate","sec":sec})
    with open('plan.txt', 'w') as f:
        f.write(str(arr))

# curl -X POST -H "Content-Type: application/json" -d '{"data":[{"mode":"mission","long":90.3584209,"lat":23.8378394},{"mode":"mission","long":90.3583164,"lat":23.8379685}]}' http://192.168.43.17:5476/plan
@app.route('/plan', methods=['POST'])
def plan():
    socketio.emit('rover',{
        'message': 'Plan received',
    })
    data = request.get_json()
    with open('plan.txt', 'w') as f:
        f.write(str(data['data']))
    return jsonify({'message': 'Plan saved'}), 200
# curl -X POST -H "Content-Type: application/json" -d '{"data":{"mode":"mission","long":12,"lat":12}}' http://
@app.route('/push', methods=['POST'])
def push():
    socketio.emit('rover',{
        'message': 'Data received',
    })
    data = request.get_json()
    arr = []
    with open('plan.txt', 'r') as f:
        data = f.read()
        arr = eval(data)
    arr.append(data['data'])
    with open('plan.txt', 'w') as f:
        f.write(str(arr))
    return jsonify({'message': 'Data pushed'}), 200
# curl -X POST http://localhost:8001/pop
@app.route('/pop', methods=['POST'])
def pop():
    socketio.emit('rover',{
        'message': 'Data popped',
    })
    arr = []
    with open('plan.txt', 'r') as f:
        data = f.read()
        arr = eval(data)
    arr.pop()
    with open('plan.txt', 'w') as f:
        f.write(str(arr))
    return jsonify({'message': 'Data popped'}), 200

def increaseIndex():
    socketio.emit('rover',{
        'message': 'Index Increased',
    })
    with open('currentIdx.txt', 'r') as f:
        idx = int(f.read())
        with open('currentIdx.txt', 'w') as f:
            f.write(str(idx+1))
def decreaseIndex():
    socketio.emit('rover',{
        'message': 'Index Decreased',
    })
    with open('currentIdx.txt', 'r') as f:
        idx = int(f.read())
        with open('currentIdx.txt', 'w') as f:
            f.write(str(idx-1))
def hasAnyVal():
    index = 0
    with open('currentIdx.txt', 'r') as f:
        index = int(f.read())
    lst = []
    with open('plan.txt', 'r') as f:
        data = f.read()
        lst = eval(data)
    if(len(lst)==index):
        return False
    return True

# curl -X POST http://localhost:8001/reset
@app.route('/reset', methods=['POST'])
def reset():
    socketio.emit('rover',{
        'message': 'Plan reset',
    })
    with open('plan.txt', 'w') as f:
        f.write("")
    with open('currentIdx.txt', 'w') as f:
        f.write("0")
    rospy.wait_for_service('mavros/set_mode')
    set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
    response = set_mode_service(custom_mode='MANUAL')
    print(response)
    rospy.wait_for_service('mavros/cmd/arming')
    arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    response = arm_service(False)
    print(response)
    rospy.wait_for_service('mavros/mission/clear')
    clear_service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
    response = clear_service()
    print(response)
    waypoints.clear()
    return jsonify({'message': 'Plan reset'}), 200
@app.route('/start', methods=['POST'])
def start():
    socketio.emit('rover',{
        'message': 'Plan started',
    })
    with open('currentIdx.txt', 'w') as f:
        f.write("0")
    if(hasAnyVal()==False):
        return jsonify({'message': 'No plan found'}), 400
    
    newMission()
    
    return jsonify({'message': 'Plan started'}), 200
# curl -X POST -H "Content-Type: application/json" -d '{"mode":"auto"}' http://localhost:8001/change_mode
@app.route('/change_mode', methods=['POST'])
def change_mode():
    socketio.emit('rover',{
        'message': 'Mode changed',
    })   
    global current_mode
    data = request.get_json()
    if(data['mode'] != 'auto' and data['mode'] != 'manual' and data['mode'] != 'arcode' and data["mode"]!="search" and data['mode']!="mission"):
        return jsonify({'error': 'Invalid mode'})
    current_mode = data['mode']
def mode_callback(msg):
    pass

def main():
    with open('plan.txt', 'w') as f:
        f.write("")
    with open('currentIdx.txt', 'w') as f:
        f.write("0")
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
    waypoints.clear()
    pub_thread = threading.Thread(target=publish_data)
    pub_thread.start()
    rospy.Subscriber('mode', String, mode_callback)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, global_position_callback)
    rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, compass_heading_callback)
    rospy.Subscriber('/mavros/rc/out', RCOut, rc_out_callback)
    rospy.Subscriber('/mavros/mission/reached', WaypointReached, mission_compelete)
    rospy.Subscriber('/aruco_go',String,arCode,queue_size=10)
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
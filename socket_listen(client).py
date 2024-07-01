import socketio
import rospy
from std_msgs.msg import String
sio = socketio.Client()
rospy.init_node('socket_listen', anonymous=True)

@sio.on('connect')
def on_connect():
    print('Connected to server')

@sio.on('disconnect')
def on_disconnect():
    print('Disconnected from server')

# @sio.on('joystick_data')
# def on_custom_event(data):
#     lst = data.split(',')
#     pub.publish("["+str(lst[0])+","+str(lst[1])+"]")
#     print(lst)

sio.connect('http://192.168.1.111:5476')
# sio.connect('http://192.168.43.17:5476')


sio.emit('zavier', {'data': 'Hello from client'})
def joystick_callback(data):
    joystick_data = data.data
    sio.emit('joystick_data', joystick_data)
rospy.Subscriber('joystick', String, joystick_callback)


# Keep the script running until manually interrupted
sio.wait()

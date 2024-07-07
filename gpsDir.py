from flask import Flask, request, jsonify
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import math
import threading
from flask_cors import CORS
import logging

app = Flask(__name__)
CORS(app)
logging.basicConfig(level=logging.DEBUG)

current_lat = 0.0
current_lon = 0.0
current_heading = 0.0
global_target_lat = 0.0
global_target_lon = 0.0

angle_pub = None
distance_pub = None
relative_heading_pub = None

def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Earth's radius in km
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c # Distance in km
    distance *= 1000 # Distance in meters
    return distance

def calculate_bearing(lat1, lon1, lat2, lon2):
    dlon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

def calculate_relative_heading(target_bearing, current_heading):
    relative_heading = target_bearing - current_heading
    return (relative_heading + 180) % 360 - 180

# curl -X POST -H "Content-Type: application/json" -d '{"lat": 37.7749, "lon": -122.4194}' http://localhost:5005/gps

@app.route('/gps', methods=['POST'])
def gps_handler():
    data = request.get_json()
    global global_target_lat, global_target_lon
    global_target_lat = data['lat']
    global_target_lon = data['lon']
    logging.debug(f"Target GPS coordinates set: lat={global_target_lat}, lon={global_target_lon}")
    
    global current_lat, current_lon, current_heading
    logging.debug(f"Current GPS coordinates: lat={current_lat}, lon={current_lon}, heading={current_heading}")
    
    distance = haversine(current_lat, current_lon, global_target_lat, global_target_lon)
    target_bearing = calculate_bearing(current_lat, current_lon, global_target_lat, global_target_lon)
    relative_heading = calculate_relative_heading(target_bearing, current_heading)
    
    logging.debug(f"Publishing: relative_heading={relative_heading}, distance={distance}")
    angle_pub.publish(relative_heading)
    distance_pub.publish(distance)
    relative_heading_pub.publish(relative_heading)
    
    return jsonify({'relative_heading': relative_heading, 'distance': distance})

def gps_callback(data):
    global current_lat, current_lon
    current_lat = data.latitude
    current_lon = data.longitude
    logging.debug(f"GPS callback: current_lat={current_lat}, current_lon={current_lon}")
    
    update_and_publish()

def heading_callback(data):
    global current_heading
    current_heading = data.data
    logging.debug(f"Heading callback: current_heading={current_heading}")
    
    update_and_publish()

def update_and_publish():
    global global_target_lat, global_target_lon, current_lat, current_lon, current_heading
    if global_target_lat == 0.0 and global_target_lon == 0.0:
        return
    
    distance = haversine(current_lat, current_lon, global_target_lat, global_target_lon)
    target_bearing = calculate_bearing(current_lat, current_lon, global_target_lat, global_target_lon)
    relative_heading = calculate_relative_heading(target_bearing, current_heading)
    
    logging.debug(f"Updating and publishing: relative_heading={relative_heading}, distance={distance}")
    angle_pub.publish(relative_heading)
    distance_pub.publish(distance)
    relative_heading_pub.publish(relative_heading)

def start_flask():
    app.run(host='0.0.0.0', port=5005)

if __name__ == '__main__':
    rospy.init_node('gps_handler_node', anonymous=True)
    
    angle_pub = rospy.Publisher('angle_topic', Float64, queue_size=10)
    distance_pub = rospy.Publisher('distance_topic', Float64, queue_size=10)
    relative_heading_pub = rospy.Publisher('relative_heading_topic', Float64, queue_size=10)
    
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, gps_callback)
    rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, heading_callback)
    
    flask_thread = threading.Thread(target=start_flask)
    flask_thread.start()
    
    rospy.spin()
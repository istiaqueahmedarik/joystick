import rospy
from std_msgs.msg import String
import time

def publish_string():
    rospy.init_node('joystick_publisher', anonymous=True)
    pub = rospy.Publisher('joystick', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    message = "[1700,1500]"
    rospy.loginfo(message)
    pub.publish(message)
    time.sleep(2)

    message="[1500,1700]"
    rospy.loginfo(message)
    pub.publish(message)
    time.sleep(2)

    message="[1700,1700]"
    rospy.loginfo(message)
    pub.publish(message)
    time.sleep(2)

    message="[1200,1200]"
    rospy.loginfo(message)
    pub.publish(message)
    time.sleep(2)
        

if __name__ == '__main__':
    try:
        publish_string()
    except rospy.ROSInterruptException:
        pass
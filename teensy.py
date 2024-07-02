import rospy
from std_msgs.msg import String

def publish_string():
    rospy.init_node('joystick_publisher', anonymous=True)
    pub = rospy.Publisher('joystick', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        message = "[1500,1500]"
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_string()
    except rospy.ROSInterruptException:
        pass
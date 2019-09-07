import rospy

class StreamrPublisher:

    def __init__(self):
        rospy.init_node("streamr_publisher_node")

        rospy.loginfo("StreamrPublisher is launched!")

    def spin(self):
        rospy.spin()


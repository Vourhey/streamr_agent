# ROS
import rospy
from std_msgs.msg import String

# AIRA
from robonomics_liability.msg import Liability
from robonomics_liability.srv import StartLiability, FinishLiability

# This package
from streamr_agent.streamr_thread import StreamrThread

class StreamrPublisher:

    def __init__(self):
        rospy.init_node("streamr_publisher_node")

        rospy.Subscriber('/liability/ready', Liability, self.on_new_liability)

        rospy.loginfo("StreamrPublisher is launched!")

    def on_new_liability(self, liability):
        rospy.loginfo('Starting process with liability: {}'.format(liability.address.address))

        th = StreamrThread(liability)
        th.run()

    def spin(self):
        rospy.spin()


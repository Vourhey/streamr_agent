import rospy

class StreamrTrader:

    def __init__(self):
        rospy.init_node("streamr_trader_node")

        rospy.loginfo("StreamrTrader node is launched!")

    def spin(self):
        rospy.spin()


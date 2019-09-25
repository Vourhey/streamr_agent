# Standart, System and Third party
from collections import namedtuple

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

        rospy.wait_for_service('/liability/finish')
        self.liability_proxy = namedtuple('liability_srvs_proxy', ['start', 'finish'])(
                                          rospy.ServiceProxy('/liability/start', StartLiability),
                                          rospy.ServiceProxy('/liability/finish', FinishLiability))


        rospy.loginfo("StreamrPublisher is launched!")

    def on_new_liability(self, liability):
        rospy.loginfo('Starting process with liability: {}'.format(liability.address.address))

        th = StreamrThread(liability)
        th.run()

        # new Thread
        # thread(liability)
        # t.run()
        #
        # Inside thread
        # pub = rospy.Publisher("/row", String)
        # liability.service.start(liability.address.address)
        # streamr = Streamr.init()
        # time_start = get_current_time()
        #
        # time_end = time_start + period
        # current_time = get_current_time()
        # while current_time <= time_end:
        #   row = get_next_row()
        #   streamr.publish(row)
        #   pub.publish(row)
        #
        #   rospy.sleep(INTERVAL)
        #   current_time = get_current_time()
        #
        # liability.service.finish()
        # die


        # self.current_liability = liability
        # self.liability_proxy.start(liability.address.address)

    def spin(self):
        rospy.spin()


# ROS
import rospy
from rosbag import Bag
from std_msgs.msg import String

from collections import namedtuple
from ipfsapi import connect
import threading
import os
from time import time
import random

# AIRA
from robonomics_liability.msg import Liability
from robonomics_liability.srv import StartLiability, FinishLiability


from streamr_agent.simple_streamr_client import SimpleStreamrClient

class StreamrThread(threading.Thread):

    INTERVAL = 5    # seconds

    def __init__(self, liability):
        threading.Thread.__init__(self)

        rospy.wait_for_service('/liability/finish')
        self.liability_proxy = namedtuple('liability_srvs_proxy', ['start', 'finish'])(
                                          rospy.ServiceProxy('/liability/start', StartLiability),
                                          rospy.ServiceProxy('/liability/finish', FinishLiability))


        self.lia = liability

        self.ipfs = connect()   # TODO probably specify the address

        objective = self._get_objective(self.lia)

        period = int(objective['/period'].data)  # seconds
        self.time_end = int(time()) + period

        stream_id = objective['/stream_id'].data
        self.streamr = SimpleStreamrClient(stream_id)

        self.email = objective['/email']

        self.pub = rospy.Publisher('/data', String, queue_size=128)

    def run(self):
        self.liability_proxy.start(self.lia.address.address)

        self._send_email()

        current_time = int(time())
        while current_time <= self.time_end:
            row = {
                'random': self._get_next_row()
            }
            self.streamr.publish(row)
            self.pub.publish(str(row))

            rospy.sleep(self.INTERVAL)
            current_time = int(time())

        self.liability_proxy.finish(self.lia.address.address)
        rospy.sleep(self.INTERVAL)
        self.pub.unregister()

    def _get_next_row(self) -> int:
        return random.randint(0, 2**64)

    def _get_objective(self, lia: Liability) -> dict:
        mhash = lia.objective.multihash
        tempdir = gettempdir()
        os.chdir(tempdir)

        # temp_obj = NamedTemporaryFile(delete=False)
        self.ipfs.get(mhash)

        messages = {}
        for topic, msg, timestamp in Bag(mhash, 'r').read_messages():
            messages[topic] = msg
        return messages

    def _send_email(self):
        pass # TODO



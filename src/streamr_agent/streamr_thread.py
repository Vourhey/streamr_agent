# ROS
import rospy
import smtplib
from rosbag import Bag
from std_msgs.msg import String

from collections import namedtuple
from ipfsapi import connect
from tempfile import gettempdir
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
        rospy.loginfo(objective)

        period = int(objective['/period'].data)  # seconds
        self.time_end = int(time()) + period

        self.stream_id = objective['/stream_id'].data
        auth_token = rospy.get_param('streamr_publisher/auth_token')
        self.streamr = SimpleStreamrClient(self.stream_id, auth_token)

        self.email = objective['/email'].data

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

        self.liability_proxy.finish(self.lia.address.address, True)
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
        rospy.loginfo("Sending an email to {}".format(self.email))
        login = rospy.get_param("streamr_publisher/login")
        email_from = rospy.get_param("streamr_publisher/email_from")
        if not email_from:
            email_from = login

        try:
            serv = smtplib.SMTP(rospy.get_param('streamr_publisher/smtp_provider'), int(rospy.get_param('streamr_publisher/smtp_port')))
            serv.ehlo()
            serv.starttls()
            # serv.ehlo()
            serv.login(login, rospy.get_param('streamr_publisher/email_password'))
        except:
            rospy.loginfo("Error while sending an email")

        liability_link = 'https://etherscan.io/address/{}#readContract'.format(self.lia.address.address)
        streamr_link = "https://{}".format(self.stream_id)
        footer = '\n--\nBest regards,\nAIRA.'
        msg = '\r\n'.join([
            'From: {}'.format(email_from),
            'To: {}'.format(self.email),
            'Subject: Streamr via Robonomics',
            '',
            'The stream was created with th id {} at liability {}.{}'
            .format(streamr_link, liability_link, footer)
        ])
        serv.sendmail(login, self.email, msg)
        rospy.loginfo("Successfully sent!")
        serv.quit()



import rospy
import requests

class SimpleStreamrClient:
    def __init__(self, stream_id, auth_token):
        self.headers = {
            'Content-Type': 'application/json',
            'authorization': 'token ' + auth_token
        }
        self.url = 'https://www.streamr.com/api/v1/streams/{}/data'.format(stream_id)

    def publish(self, data: dict):
        response = requests.post(self.url, data=data, headers=self.headers)
        rospy.loginfo(response)


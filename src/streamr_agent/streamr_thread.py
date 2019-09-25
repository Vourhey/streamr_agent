# ROS
from rosbag import Bag

import threading

class StreamrThread(threading.Thread):
    def __init__(self, liability):
        threading.Thread.__init__(self)
        self.lia = liability

        objective = self._get_objective(self.lia)

        period = int(objective['/period'].data)  # seconds


    def run():
        pass

    def _get_objective(self, lia):
        mhash = lia.objective.multihash
        tempdir = gettempdir()
        os.chdir(tempdir)

        # temp_obj = NamedTemporaryFile(delete=False)
        self.ipfs.get(mhash)

        messages = {}
        for topic, msg, timestamp in Bag(mhash, 'r').read_messages():
            messages[topic] = msg
        return messages



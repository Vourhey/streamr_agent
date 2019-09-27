import rospy
import requests
import os
import signal
from ipfsapi import connect
from tempfile import gettempdir, NamedTemporaryFile
from rosbag import Bag
from std_msgs.msg import String
from robonomics_msgs.msg import Demand
from ipfs_common.msg import Multihash
from ethereum_common.msg import Address, UInt256
from ethereum_common.srv import BlockNumber

MODEL = 'QmbpGnAyjNeNabUg8gaxTUKDPcozCRDd2JaVck7uHXDu9g'
TOKEN = '0x15bb868397d03Aab342db7d0bb2D3A83336699c1'
LIGHTHOUSE = '0x202a09A451DE674d2d65Bf1C90968a8d8F72cf7b'
VALIDATOR = '0x0000000000000000000000000000000000000000'
AUTH_TOKEN = 'bFV-LnI1RC66Y7xfWGGK8QkwZ50I-uTgCQx8Z-5lcyTg'
MULTIPLIER_COST = 10**12     # cost for 1 sec
TERMINATE = False

def make_deadline() -> str:
    lifetime = 100
    deadline = rospy.ServiceProxy('/eth/current_block', BlockNumber)().number + lifetime
    return str(deadline)

def make_demand(objective: str, cost: int) -> Demand:
    rospy.loginfo('Making demand...')

    demand = Demand()
    demand.model = Multihash(MODEL)
    demand.objective = Multihash(objective)
    demand.token = Address(TOKEN)
    demand.cost = UInt256(str(cost))
    demand.lighthouse = Address(LIGHTHOUSE)
    demand.validator = Address(VALIDATOR)
    demand.validatorFee = UInt256('0')
    demand.deadline = UInt256()
    demand.deadline.uint256 = make_deadline()

    rospy.loginfo(demand)
    return demand

def calculate_period(choice: int) -> str:
    switcher = {
        1: 10,
        2: 60,
        3: 10*60
    }

    period = switcher.get(choice, "Invalide choice")
    return str(period)

def calculate_cost(choice: int) -> int:
    if choice == 1:
        return MULTIPLIER_COST * 10
    elif choice == 2:
        return MULTIPLIER_COST * 60
    elif choice == 3:
        return MULTIPLIER_COST * 10 * 60

    rospy.logerr("Invalide choice")

def create_new_stream() -> str:
    url = 'https://www.streamr.com/api/v1/streams'
    headers = {
        'Content-Type': 'application/json',
        'authorization': 'token ' + AUTH_TOKEN
    }
    body = {
        "name": "Robonomics Stream",
        "description": "Test robonomics stream",
        "config": {
        "fields": [{
            "name": "random",
            "type": "number"
        }]
        }
    }
    response = requests.post(url, json=body, headers=headers)
    response = response.json()
    rospy.loginfo("Created a new stream: {}".format(response))
    return response['id']

def create_objective(period: str, stream_id: str, email: str) -> str:
    ipfs = connect()
    tempdir = gettempdir()
    os.chdir(tempdir)

    with NamedTemporaryFile(delete=False) as f:
        bag = Bag(f.name, 'w')
        bag.write('/period', String(period))
        bag.write('/stream_id', String(stream_id))
        bag.write('/email', String(email))
        bag.close()

        res = ipfs.add(f.name)
        rospy.loginfo("Hash of the objective is {}".format(res['Hash']))
        return res['Hash']

def subscribe_to_stream(stream_id: str):
    rospy.loginfo("Subscribe to the stream {}".format(stream_id))
    url = "https://www.streamr.com/api/v1/streams/{}/data/partitions/0/last?count=1".format(stream_id)
    headers = {
        "Content-Type": "application/json",
        "authorization": "token " + AUTH_TOKEN
    }
    while not TERMINATE:
        response = requests.get(url, headers=headers)
        print(response.json())
        rospy.sleep(5)

def exit_gracefully(signum, frame):
    global TERMINATE
    rospy.loginfo("exit_gracefully")
    TERMINATE = True

def main():
    rospy.init_node("promisee_node")

    signal.signal(signal.SIGTERM, exit_gracefully)
    signal.signal(signal.SIGINT, exit_gracefully)

    pub = rospy.Publisher('/liability/infochan/eth/signing/demand', Demand, queue_size=128)

    email = input("Enter your email (vadim.razorq@gmail.com): ") or 'vadim.razorq@gmail.com'
    rospy.loginfo("Email is {}".format(email))

    print("Period:\n1. 10 sec\n2. 1 min\n3. 10 min")
    choice = int(input("Choose the period [1, 2, 3] (1): ") or 1)
    period = calculate_period(choice)
    rospy.loginfo("Period is {}".format(period))

    cost = calculate_cost(choice)
    rospy.loginfo("Cost is {}".format(cost))

    stream_id = create_new_stream()
    rospy.loginfo("Stream id is {}".format(stream_id))

    rospy.loginfo("Creating objective...")
    objective = create_objective(period, stream_id, email)

    demand = make_demand(objective, cost)
    pub.publish(demand)

    subscribe_to_stream(stream_id)

if __name__ == "__main__":
    main()


import rospy

# Robonomics communication
from robonomics_msgs.msg import Offer, Demand
from ethereum_common.msg import Address, UInt256
from ethereum_common.srv import Accounts, BlockNumber

class StreamrTrader:

    def __init__(self):
        rospy.init_node("streamr_trader_node")
        rospy.loginfo('Launching streamr trader node...')

        rospy.wait_for_service('/eth/current_block')
        rospy.wait_for_service('/eth/accounts')
        self.accounts = rospy.ServiceProxy('/eth/accounts', Accounts)()
        rospy.loginfo(str(self.accounts))  # AIRA ethereum addresses

        rospy.Subscriber('/liability/infochan/incoming/demand', Demand, self.on_incoming_demand)

        self.signing_offer = rospy.Publisher('/liability/infochan/eth/signing/offer', Offer, queue_size=128)

        self.MODEL = rospy.get_param("~model")
        self.TOKEN = rospy.get_param("~token")

        rospy.loginfo("StreamrTrader node is launched!")

    def on_incoming_demand(self, incoming: Demand):
        rospy.loginfo('Incoming demand %s...', str(incoming))
        if (incoming.model.multihash == self.MODEL and incoming.token.address == self.TOKEN):
            rospy.loginfo('For my model and token!')
            self.make_offer(incoming)
        else:
            rospy.loginfo('It doesn\'t fit my model or token, skip it.')

    def make_deadline(self):
        lifetime = int(rospy.get_param('~order_lifetime'))
        deadline = rospy.ServiceProxy('/eth/current_block', BlockNumber)().number + lifetime
        return str(deadline)

    def make_offer(self, incoming):
        rospy.loginfo('Making offer...')

        offer = Offer()
        offer.model = incoming.model
        offer.objective = incoming.objective
        offer.token = incoming.token
        offer.cost = incoming.cost
        offer.lighthouse = Address()
        offer.lighthouse.address = rospy.get_param('~lighthouse')
        offer.validator = incoming.validator
        offer.lighthouseFee = UInt256()
        offer.lighthouseFee.uint256 = '0'
        offer.deadline = UInt256()
        offer.deadline.uint256 = self.make_deadline()

        self.signing_offer.publish(offer)
        rospy.loginfo(offer)

    def spin(self):
        rospy.spin()


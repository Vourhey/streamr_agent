from .streamr_trader import StreamrTrader
from .streamr_publisher import StreamrPublisher

def trader_node():
    StreamrTrader().spin()

def publisher_node():
    StreamrPublisher().spin()


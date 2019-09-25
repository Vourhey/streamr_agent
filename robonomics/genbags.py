#!/usr/bin/env python3
# Generates few rosbag files for test purpose
from rosbag import Bag
from std_msgs.msg import String

### Create rosbag files

topic = "/period"

bag_2min = Bag("objective_2min.bag", "w")
two_minutes = str(2 * 60)
bag_2min.write(topic, String(two_minutes))
bag_2min.close()

bag_1h = Bag("objective_1h.bag", "w")
one_hour = str(60 * 60)
bag_1h.write(topic, String(one_hour))
bag_1h.close()

bag_1d = Bag("objective_1d.bag", "w")
one_day = str(24 * 60 * 60)
bag_1d.write(topic, String(one_day))
bag_1d.close()

### Publish to IPFS
from ipfsapi import connect

ipfs_client = connect()
response = ipfs_client.add(bag_2min.filename)
print("{}: {}".format(response['Name'], response['Hash']))

response = ipfs_client.add(bag_1h.filename)
print("{}: {}".format(response['Name'], response['Hash']))

response = ipfs_client.add(bag_1d.filename)
print("{}: {}".format(response['Name'], response['Hash']))


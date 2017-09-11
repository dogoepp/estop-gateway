#!/usr/bin/env python
# coding: utf-8

import socket

# Ros-related
import rospy
from std_msgs.msg import UInt32


class HeartBeatGateway:
    def __init__(self, port, timeout = 0.1, topic='pulse', queue_size=10):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # TODO: have a look to what can be done with `protocol`

        host = socket.gethostname() # get local machine name
        rospy.loginfo("Host : " + str(host))

        self.socket.bind(('', port)) # bind the socket to the port
        # self.socket.settimeout(timeout) # set a timeout for the socket

        self.publisher = rospy.Publisher(topic, UInt32, queue_size=queue_size)

        self.suivant = 1 # seed for the pseudo-random number generator

    def recieve_tick(self):
        data = ''
        addr = ''
        try:
            data, addr = self.socket.recvfrom(1024) # buffer size is 1024 bytes
        except socket.timeout as e:
            pass # timeout reached and no data arrived
        except socket.error as e:
            rospy.logerr("Socket reading triggered the following error: {}"
                .format(e))

        return data

    def relay_tick(self, data):
        try:
            rospy.loginfo(repr(int(data.decode().strip('\0'))))
            message = UInt32(int(data.decode().strip('\0')))
            self.publisher.publish(message)
        except ValueError as e:
            rospy.logwarn(e)
        except Exception as e:
            rospy.logerr("There's an error: {}".format(e))



if __name__ == '__main__':
    try:
        rospy.init_node('talker')
        s = HeartBeatGateway(1042, 0.1)
        # rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            data = s.recieve_tick()

            s.relay_tick(data)

            # rate.sleep()
    except rospy.ROSInterruptException:
        pass
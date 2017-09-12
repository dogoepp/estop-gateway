#!/usr/bin/env python
# coding: utf-8

# network communication
import socket
import struct
# for Time-based One-Time Password
import hashlib
import hmac

# Ros-related
import rospy
from std_msgs.msg import UInt32

def bytes_to_str(data):
    string = ''
    data = bytearray(data)
    for byte in data:
        string = string + hex(byte)[2:]
    return string

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
        seconds = 0
        millis = 0
        addr = ''
        decoded_data = 0
        try:
            data, addr = self.socket.recvfrom(1024) # buffer size is 1024 bytes
            data = bytearray(data)
            if len(data) <> 40:
                rospy.logerr("Data received should have 40 bytes (32 bytes for the hash and 2*4 for the time). Received {0} bytes instead.".format(len(data)))
            else:
                # rospy.logdebug("time field of the bare data: "+bytes_to_str(data[32:]))
                try:
                    # (data, seconds, millis)= struct.unpack('<32sII', data)
                    decoded_data= struct.unpack('<32s8s', data)
                    # rospy.logdebug("Data received: {0}, {1}, {2}".format(data, seconds, millis))
                except struct.error as e:
                    rospy.logerr("Unpacking error: " + str(e))
            # string = ''
            # for byte in data:
            #     string = string + hex(byte)[2:]
            # data = string
        except socket.timeout as e:
            pass # timeout reached and no data arrived
        except socket.error as e:
            rospy.logerr("Socket reading triggered the following error: {}"
                .format(e))
            raise e

        return decoded_data

    def relay_tick(self, data):
        try:
            rospy.loginfo(repr(int(data.decode().strip('\0'))))
            message = UInt32(int(data.decode().strip('\0')))
            self.publisher.publish(message)
        except ValueError as e:
            rospy.logwarn(e)
        except Exception as e:
            rospy.logerr("There's an error: {}".format(e))

    def relay_print(self, data):
        try:
            rospy.loginfo(data)
            # rospy.loginfo(type(data))
        except Exception as e:
            rospy.logerr("There's an error: {}".format(e))

    def check_data(self, data):
        # rospy.loginfo(str(time))
        # rospy.loginfo(bytes_to_str(hash_bytes))
        same = self.check_hmac(data)
        if same:
            rospy.loginfo("the messages are the same")
        else:
            rospy.loginfo("the messages differ")

    def check_hmac(self, data):
        key = b"16:40:35"
        rospy.logdebug("time is: " + bytes_to_str(data[1]))
        rospy.logdebug("hash is: " + bytes_to_str(data[0]))
        h = hmac.new(key, str(data[1]), hashlib.sha256)
        # h.update(str(millis))
        rospy.logdebug("local hash is: "+bytes_to_str(h.digest()))
        return hmac.compare_digest(h.digest(), str(data[0]))


if __name__ == '__main__':
    try:
        rospy.init_node('talker', log_level=rospy.DEBUG)
        s = HeartBeatGateway(1042, 0.1)
        # rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            data = s.recieve_tick()

            # s.relay_tick(data)
            # s.relay_print(data)
            s.check_data(data)

            # rate.sleep()
    except rospy.ROSInterruptException:
        pass
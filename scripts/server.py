#!/usr/bin/env python
# coding: utf-8

# network communication
import socket
import struct
# for Time-based One-Time Password
import hashlib
import hmac

import math
import time

import collections # for the circular buffer deque

# Ros-related
import rospy
from std_msgs.msg import UInt32

def bytes_to_str(data):
    string = ''
    data = bytearray(data)
    for byte in data:
        string = string + hex(byte)[2:]
    return string

def median(list):
    median = None
    sorted_list = sorted(list)
    size = len(list)

    if size % 2 == 0: # Even number of elements
        median = (sorted_list[size/2-1] + sorted_list[size/2]) / 2
    else:             # Odd number of elements
        median = sorted_list[size//2]

    return median

class HeartBeatGateway(object):
    """
        Relay heartbeat from UDP sockets to ROS.

        The method :func:`recieve_tick` waits (blocking) for a message to arrive on a given port, parses it and returns the parsed data.

        .. note:: We sometimes talk about pulse, instead of heartbeat. We mean the same thing.

        TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
        This class will eventually only handle the reception, parsing and checking of data.
        The rest will be caried out by a class handling the ROS node.
        TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
    """
    def __init__(self, port, max_delay, key, source_ip,
                 timeout = 0.1, topic='pulse', queue_size=1):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.socket.bind(('', port)) # bind the socket to the port
        # TODO: self.socket.settimeout(timeout) # set a timeout for the socket

        self.source_ip = source_ip
        self.max_delay = max_delay
        self.key = key

        self.publisher = rospy.Publisher(topic, UInt32, queue_size=queue_size)

        # parser for incoming data
        self.struct = struct.Struct('<32s8sf')

    def __call__(self, sliding_window = (lambda x: x)):
        try:
            tick = s.recieve_tick()
            rospy.logdebug("received tick")
            if s.check_data(tick):
                rospy.logdebug("received tick and checked")
                battery_level = tick['decoded'][2]
                # We have to rely on a smoothing method because the ADC
                # (analog to digital converter) measurements are noisy, as
                # discussed in https://github.com/esp8266/Arduino/issues/2070
                battery_level = sliding_window(battery_level)

                # s.relay_tick(data)
                # s.relay_print(data)

                rospy.loginfo("correct tick received")
                rospy.loginfo("Battery level: {0}".format(battery_level))
                rospy.loginfo("             : {0}".format(tick['decoded'][2]))
            else:
                rospy.lofingo("checking failed")
        except socket.error:
            pass

    def recieve_tick(self):
        addr = ''
        decoded_data = 0
        try:
            data, addr = self.socket.recvfrom(1024) # buffer size is 1024 bytes
            data = bytearray(data)
            if len(data) <> self.struct.size:
                rospy.logfatal("Data received should have {0} bytes (32 bytes for "
                             "the hash, 2*4 for the time, 4 for the battery "
                             "level). Received {1} bytes instead."
                             .format(self.struct.size, len(data)))
            else:
                try:
                    decoded_data= self.struct.unpack(data)
                except struct.error as e:
                    rospy.logerr("Unpacking error: " + str(e))
        except socket.timeout as e:
            pass # timeout reached and no data arrived
        except socket.error as e:
            rospy.logerr("Socket reading triggered the following error: {}"
                .format(e))
            raise e

        return {"decoded": decoded_data, "address":addr}

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
        address = data['address'][0]
        decoded_data = data['decoded']

        # Check message source (IP address)
        if address != self.source_ip:
            rospy.logdebug("The IP address is incorrect ({0}, {1} expected)"
                           .format(address, self.source_ip))
            return False

        # Check that the message is not too old
        # in decoded_data, seconds and milliseconds are not separated, we do
        # this separation here and take only the seconds.
        (timestamp,) = struct.unpack('<I', decoded_data[1][0:4])
        local_timestamp = int(math.floor(time.time()))
        if abs(timestamp - local_timestamp) > self.max_delay:
            rospy.logdebug("The received tick is time-shifted (by {0} seconds)"
                           .format(timestamp - local_timestamp))
            return False

        # Check the HMAC (TOPT)
        if not(self._check_hmac(data['decoded'])):
            rospy.logdebug("The hash of the tick is not ok")
            return False

        return True

    def _check_hmac(self, data):
        h = hmac.new(self.key, str(data[1]), hashlib.sha256)
        return hmac.compare_digest(h.digest(), str(data[0]))

class SlidingWindow(object):
    """ Store a fixed number of past data and apply to them a given function.

        Our current usage is to store ten past values and either get the min of
        them or the average.
    """
    def __init__(self, operator, size):
        self.window = collections.deque(maxlen=size)
        self.operator = operator

    def __call__(self, data=None):
        if data is not None:
            self.window.append(data)
        return self.operator(self.window)

    def compute(self, data):
        return self.__apply__(data)

    def append(self, data):
        self.window.append(data)


if __name__ == '__main__':
    try:
        rospy.init_node('e_stop', anonymous=True, log_level=rospy.DEBUG)
        max_delay = 2 # seconds
        key = b"16:40:35"
        source_ip = '152.81.10.184'
        s = HeartBeatGateway(1042, max_delay, key, source_ip, 0.1)
        sliding_min = SlidingWindow(median, 50)

        # rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            s(sliding_min)
            # try:
            #     tick = s.recieve_tick()
            #
            #     # s.relay_tick(data)
            #     # s.relay_print(data)
            #     if s.check_data(tick):
            #         battery_level = tick['decoded'][2]
            #         # We have to rely on a smoothing method because the ADC
            #         # (analog to digital converter) measurements are noisy, as
            #         # discussed in https://github.com/esp8266/Arduino/issues/2070
            #         battery_level = sliding_min(battery_level)
            #         rospy.loginfo("correct tick received")
            #         rospy.loginfo("Battery level: {0}".format(battery_level))
            # except socket.error:
            #     pass

            # rate.sleep()
    except rospy.ROSInterruptException:
        pass
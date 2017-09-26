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

import logging

import collections  # for the circular buffer deque


def bytes_to_str(data):
    """
    Represents the bytes in `data` as a string of hexadecimal pairs.

    Exemple output: "2AFC" for an input which first byte corresponds to 0x2A
        and the second 0xFC

    :param data: string of bytes
    :return: string representing `data` in hexadecimal
    """

    string = ''
    data = bytearray(data)
    for byte in data:
        string = string + hex(byte)[2:]
    return string


def median(list):
    median = None
    sorted_list = sorted(list)
    size = len(list)

    if size % 2 == 0:  # Even number of elements
        median = (sorted_list[size/2-1] + sorted_list[size/2]) / 2
    else:              # Odd number of elements
        median = sorted_list[size//2]

    return median


class HeartBeatGateway(object):
    """
        Relay heartbeat from UDP sockets to ROS.

        The method :func:`receive_tick` waits (blocking) for a message to
        arrive on a given port, parses it and returns the parsed data.

        .. note:: We sometimes talk about pulse, instead of heartbeat. We mean
            the same thing.
    """

    def __init__(self, port, max_delay, key, source_ip,
                 timeout=0.1):
        # Setup logging
        self._logger = logging.getLogger(__name__)
        self._logger.addHandler(logging.NullHandler())

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(('', port))  # bind the socket to the port
        # TODO: self._socket.settimeout(timeout) # set a timeout for the socket
        self._logger.debug("Listening on port {}".format(port))

        self.source_ip = source_ip
        self.max_delay = max_delay
        self.key = key

        # parser for incoming data
        self.struct = struct.Struct('<32s8sf')

    def receive_tick(self):
        """
        Wait for a message from the emergency stop, and process it.

        .. note:: This method is blocking.

        When a new frame arrives, we check that it has the right size. Then its
        data are unpacked and returned, along with the sender's IP address.

        :return: dictionnary of the shape
            `{"decoded": decoded_data, "address":addr}`
            where `decoded_data` is a tuple with a 32-bytes string, an 8-bytes
            string and a float; `addr` represents the sender's address, as
            returned by methods of the `socket` module.
        """

        addr = ''
        decoded_data = 0

        try:
            data, addr = self._socket.recvfrom(1024)  # buffer size is 1024 B
            data = bytearray(data)
            # Did we receive the right number of bytes ?
            if len(data) == self.struct.size:
                try:
                    decoded_data = self.struct.unpack(data)
                except struct.error as e:
                    self._logger.error("The following error arrised on "
                                       "processing (struct.unpack) a message "
                                       "from emergency stop: " + str(e))
                    return {"decoded": None, "address": addr}
            else:
                self._logger.warn("Received {0} bytes instead of {1} bytes "
                                  "(32 bytes for the hash, 2*4 for the time, "
                                  "4 for the battery level)."
                                  .format(len(data), self.struct.size))
                return {"decoded": None, "address": addr}
        except socket.timeout as e:  # TODO: remove if timeout not used
            pass  # timeout reached and no data arrived
        except socket.error as e:
            # Error number 4 is for interrupted system call, which happends
            # when we close the programm. It is not a problem
            if 4 != e.errno:
                self._logger.error("Socket reading triggered the following "
                                   "error: {}".format(e))
            return {"decoded": None, "address": ('', '')}

        return {"decoded": decoded_data, "address": addr}

    def relay_tick(self, data):
        try:
            self._logger.info(repr(int(data.decode().strip('\0'))))
            message = UInt32(int(data.decode().strip('\0')))
            self.publisher.publish(message)
        except ValueError as e:
            self._logger.warn(e)
        except Exception as e:
            self._logger.error("There's an error: {}".format(e))

    def relay_print(self, data):
        try:
            self._logger.info(data)
            # self._logger.info(type(data))
        except Exception as e:
            self._logger.error("There's an error: {}".format(e))

    def check_data(self, data):
        """
        Check that the message received is valid.

        This implies three tests:
        * sender IP address
        * authentication of the sender (with HMAC)
        * time discrepency between emission and reception of the message not
            too big

        :param data: output of :func:`receive_tick`
        :return: None if the data has no content,
                 True when all tests pass
        """

        address = data['address'][0]
        decoded_data = data['decoded']

        # Check that we received valid data
        if decoded_data is None:
            return None

        # Check message source (IP address)
        if address != self.source_ip:
            self._logger.debug("The IP address is incorrect ({0}, "
                               "expected {1})".format(address, self.source_ip))
            return False

        # Authenticate the sender, by checking the HMAC (TOPT)
        if not self._check_hmac(decoded_data):
            self._logger.debug("The hash of the tick is not ok")
            return False

        # Check that the message is not too old or too young (in seconds)
        diff = self._compare_time(decoded_data)
        if abs(diff) > self.max_delay:
            self._logger.info("The received tick is time-shifted "
                              "(by {0} seconds)".format(diff))
            return False

        return True

    def _check_hmac(self, data):
        """
        Check that the message's hmac hash is correct.

        Firstly, we compute the hmac of the message's payload, using the
        shared secret key. Then, we compare this hash with the one in the
        message.

        :param data: "decoded" entry in the dictionary returned by
            `receive_tick`
        :return: True when the two hashes match
        """

        h = hmac.new(self.key, str(data[1]), hashlib.sha256)
        return hmac.compare_digest(h.digest(), str(data[0]))

    def _compare_time(self, decoded_data):
        """
        Compute the difference between the message timestamp and the local
        timestamp.

        To do so, we extract the time in `decoded_data` and get the OS's time
        (in UTC). The computation is based on timestamps to the resolution of
        one second.

        :param data: the output of receive_tick
        :return: difference between time in the message and host's time, in
            seconds
        """

        # in decoded_data, seconds and milliseconds are not separated, we do
        # this separation here and take only the seconds.
        try:
            (timestamp,) = struct.unpack('<I', decoded_data[1][0:4])
        except TypeError:
            return False

        # This timestamp is local in the sense that it is retrieved from the
        # host OS.
        local_timestamp = int(math.floor(time.time()))

        return timestamp - local_timestamp


class SlidingWindow(object):
    """ Store a fixed number of past data and apply to them a given function.

        Our current usage is to store ten past values and either get the min of
        them or the average.

        TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
        make this class a subclass of collections.deque
        TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
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

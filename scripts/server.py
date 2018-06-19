#!/usr/bin/env python2
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

    Exemple output: "2a:fc" for an input which first byte corresponds to 0x2A
        and the second 0xFC

    :param data: string of bytes
    :return: string representing `data` in hexadecimal
    """

    return ":".join("{:02x}".format(ord(c)) for c in data)


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

        The method :func:`receive_heartbeat` waits (blocking) for a message to
        arrive on a given port, parses it and returns the parsed data.

        .. note:: We sometimes talk about pulse, instead of heartbeat. We mean
            the same thing.
    """

    def __init__(self, port, max_delay, key, relay_key, source_ip,
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
        self.relay_key = relay_key

        # parser for incoming data
        self.struct = struct.Struct('<32sIHf')

        self.previous_count = 0
        self.previous_s = 0

    def receive_heartbeat(self):
        """
        Wait for a message from the emergency stop, and process it.

        .. note:: This method is blocking.

        When a new frame arrives, we check that it has the right size. Then its
        data are unpacked and returned, along with the sender's IP address.

        :return: dictionnary with keys 'hash', 's', 'count', 'charge' and
            'address', unless there was a problem with the message. In that
            case, some fields might be missing. Types of the fields :
            - 'hash' is a 32-bytes string
            - 's' and 'count' 4-bytes and 2-bytes positive integers
            - 'rpayload' is the raw 6-byte concatentation of the two variables used
                to compute the hash
            - 'charge' is a float
            - `address` represents the sender's address, as returned by methods of
                the `socket` module.
        """

        decoded_data = {}

        try:
            data, decoded_data["address"] = self._socket.recvfrom(1024)  # buffer size is 1024 B
            data = bytearray(data)
            self.unpack_data(decoded_data, data)
            decoded_data['rpayload'] = data[32:38]
        except socket.timeout as e:  # TODO: remove if timeout not used
            pass  # timeout reached and no data arrived
        except socket.error as e:
            # Error number 4 is for interrupted system call, which happends
            # when we close the programm. It is not a problem
            if 4 != e.errno:
                self._logger.error("Socket reading triggered the following "
                                   "error: {}".format(e))

        return decoded_data

    def unpack_data(self, decoded_data, data):
        # Did we receive the right number of bytes ?
        if len(data) == self.struct.size:
            try:
                decoded = self.struct.unpack(data)
                # put the unpacked data in a dictionary
                field_names = ("hash", "s", "count", "charge")
                decoded_data.update(dict(zip(field_names, decoded)))

                self._logger.debug("Received data:\n\thash {},\n\ttime {} s,"
                                    "\n\tcount {}\n\tcharge {} %".format(
                                        bytes_to_str(decoded_data['hash']),
                                        decoded_data['s'],
                                        decoded_data['count'],
                                        decoded_data['charge']
                                    ))
            except struct.error as e:
                self._logger.error("The following error arrised on "
                                   "processing (struct.unpack) a message "
                                   "from emergency stop: " + str(e))
        else:
            self._logger.warn("Received {0} bytes instead of {1} bytes "
                              "(32 bytes for the hash, 2*4 for the time, "
                              "4 for the battery level)."
                              .format(len(data), self.struct.size))

    def relay_heartbeat(self, data):
        payload = struct.pack('<IH', data['s'], data['count'])
        data['relay_hash'] = hmac.new(self.relay_key, payload, hashlib.sha256).digest()
        return data

    def relay_print(self, data):
        try:
            self._logger.info(data)
            # self._logger.info(type(data))
        except Exception as e:
            self._logger.error("There's an error: {}".format(e))

    def check_data(self, data):
        """
        Check that the message received is valid.

        This implies four tests:
        * sender IP address
        * authentication of the sender (with HMAC)
        * time discrepency between emission and reception of the message not
            too big
        * the message has never been seen before

        :param data: output of :func:`receive_heartbeat`
        :return: None if the data has no content,
                 True when all tests pass
        """

        address = data['address'][0]

        # Check that we received valid data
        if data is None:
            return None

        # Check message source (IP address)
        if address != self.source_ip:
            self._logger.warn("The sender's IP address is incorrect ({0}, "
                               "accepted {1}).".format(address, self.source_ip))
            return False

        # Authenticate the sender, by checking the HMAC (TOPT)
        if not self._check_hmac(data):
            self._logger.warn("The hash of the heartbeat is not ok.")
            return False

        # Check that the message is not too old or too young (in seconds)
        diff = self._compare_time(data)
        if abs(diff) > self.max_delay:
            self._logger.warn("The received heartbeat is time-shifted "
                              "(by {0} seconds).".format(diff))
            return False

        # Check that the message was not received twice
        if not self._check_count_increment(data, self.previous_count, self.previous_s):
            self._logger.warn("The counter of the received heartbeat is "
                "incorrect")
            return False

        self.previous_count = int(data['count'])
        self.previous_s = int(data['s'])

        return True

    def _check_hmac(self, data):
        """
        Check that the message's hmac hash is correct.

        Firstly, we compute the hmac of the message's payload, using the
        shared secret key. Then, we compare this hash with the one in the
        message.

        :param data: data provided by `receive_heartbeat`
        :return: True when the two hashes match
        """

        if 'rpayload' in data:
            time = data['rpayload']
            h = hmac.new(self.key, time, hashlib.sha256)
            return hmac.compare_digest(h.digest(), str(data['hash']))
        else:
            return False

    def _compare_time(self, data):
        """
        Compute the difference between the message timestamp and the local
        timestamp.

        To do so, we extract the time in `decoded_data` and get the OS's time
        (in UTC). The computation is based on timestamps to the resolution of
        one second.

        :param data: the output of `receive_heartbeat`
        :return: difference between time in the message and host's time, in
            seconds
        """

        # This timestamp is local in the sense that it is retrieved from the
        # host OS.
        local_timestamp = int(math.floor(time.time()))

        if 's' in data:
            timestamp = data['s']
            return timestamp  - local_timestamp
        else:
            return local_timestamp

    def _check_count_increment(self, data, previous_count, previous_s):
        """
        Check that the 'count' field is bigger than the previous value.

        The field 'count' is reset to 0 at each new second. Until the next
        second, all messages should have an increasing value for 'count'.

        This method makes sure of that.

        :param data: data provided by `receive_heartbeat`
        :param previous_count: last 'count' value
        :param previous_s: value of 's' that was given in the last valid
            heartbeat
        :return: True when we respect the above-described rule
        """
        current_s = int(data['s'])

        # We cannot accept to go back in time
        if current_s >= previous_s:
            min_count = 0 if current_s > previous_s else previous_count + 1
            # For a given second, the counter must be increasing
            if data['count'] >= min_count:
                return True
            else:
                return False
        else:
            return False

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

    # TODO: remove this
    # def compute(self, data):
    #     return self.__apply__(data)

    def append(self, data):
        self.window.append(data)


if __name__ == "__main__":
    # Calls to make the logs appear in the terminal
    sh = logging.StreamHandler()
    logging.getLogger('__main__').addHandler(sh)
    # logging.getLogger('__main__').setLevel(logging.DEBUG)

    max_delay = 2  # seconds
    key = b"16:40:35"
    relay_key = b"08:36:55"
    # source_ip = '152.81.10.184'
    source_ip = '152.81.70.17'
    sliding_window = SlidingWindow(median, 50)

    server = HeartBeatGateway(1042, max_delay, key, relay_key, source_ip, 0.1)
    while(True):
        try:
            heartbeat = server.receive_heartbeat()

            # Check that we did receive data
            if "hash" in heartbeat:
                # Check the data's validity
                if server.check_data(heartbeat):
                    battery_level = heartbeat['charge']
                    # We have to rely on a smoothing method because the ADC
                    # (analog to digital converter) measurements are noisy, as
                    # discussed in
                    # https://github.com/esp8266/Arduino/issues/2070
                    battery_level = sliding_window(battery_level)

                    print("correct heartbeat received")
                    print("Battery level: {0}".format(battery_level))
                    print("             : {0}"
                                  .format(heartbeat['charge']))
                else:
                    print("invalid heartbeat received")
        except socket.error:
            pass
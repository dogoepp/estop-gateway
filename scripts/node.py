#!/usr/bin/env python
# coding: utf-8

# Standard
import logging
import socket

# Ros-related
import rospy
from estop_gateway_udp.msg import Heartbeat

# Project imports
import server


class GatewayNode(object):
    def __init__(self, topic='~heartbeat', queue_size=1, anonymous=False):
        rospy.init_node('estop_heartbeat_udp', anonymous=anonymous, log_level=rospy.DEBUG)
        max_delay = 2  # seconds
        key = b"16:40:35"
        relay_key = b"08:36:55"
        # source_ip = '152.81.10.184'
        source_ip = '152.81.70.17'
        self.server = server.HeartBeatGateway(1042, max_delay,
                                              key, relay_key, source_ip, 0.1)
        self.sliding_window = server.SlidingWindow(server.median, 50)

        self.publisher = rospy.Publisher(topic, Heartbeat, queue_size=queue_size)

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.process_message()
        except rospy.ROSInterruptException:
            pass

    def process_message(self):
        try:
            heartbeat = self.server.receive_heartbeat()

            # Check that we did receive data
            if len(heartbeat) > 1:
                rospy.logdebug("received heartbeat")

                # Check the data's validity
                if self.server.check_data(heartbeat):
                    battery_level = heartbeat['charge']
                    # We have to rely on a smoothing method because the ADC
                    # (analog to digital converter) measurements are noisy, as
                    # discussed in
                    # https://github.com/esp8266/Arduino/issues/2070
                    battery_level = self.sliding_window(battery_level)

                    self.relay_heartbeat(heartbeat)
                    # self.send_battery_status()

                    rospy.logdebug("correct heartbeat received")
                    rospy.logdebug("Battery level: {0}".format(battery_level))
                    rospy.logdebug("             : {0}"
                                  .format(heartbeat['charge']))
                else:
                    rospy.logwarn("An invalid heartbeat has been received: "
                        "source IP: {ip} "
                        "seconds: {s} "
                        "counter: {c} "
                        "hash: {h}.".format(
                            ip=heartbeat['address'][0], s=heartbeat['s'],
                            c=heartbeat['count'],
                            h=server.bytes_to_str(heartbeat['hash'])
                            ))
        except socket.error:
            pass

    def relay_heartbeat(self, heartbeat):
        self.server.relay_heartbeat(heartbeat)
        message = Heartbeat()
        message.header.stamp = rospy.Time(heartbeat['s'])
        message.header.frame_id = "0"
        message.count = heartbeat['count']
        message.hash = heartbeat['relay_hash']
        # TODO: This logging call is a hack, remove it
        # self.server._logger.info("Sending heartbet:\n\thash {},\n\ttime {} s,"
        #                     "\n\tcount {}".format(
        #                         server.bytes_to_str(heartbeat['relay_hash']),
        #                         heartbeat['s'],
        #                         heartbeat['count']
        #                     ))
        self.publisher.publish(message)

class ConnectPythonLoggingToROS(logging.Handler):
    """
    Class interfacing logs using Python's standard logging facility and ROS's
    way of logging. It is meant to be used as a Handler for the logging module.

    Source: https://gist.github.com/nzjrs/8712011
    """

    MAP = {
        logging.DEBUG:    rospy.logdebug,
        logging.INFO:     rospy.loginfo,
        logging.WARNING:  rospy.logwarn,
        logging.ERROR:    rospy.logerr,
        logging.CRITICAL: rospy.logfatal
    }

    def emit(self, record):
        try:
            self.MAP[record.levelno]("%s: %s" % (record.name, record.msg))
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno,
                         record.name, record.msg))


if __name__ == '__main__':
    # reconnect logging calls which are children of this to the ros log system
    logging.getLogger('server').addHandler(ConnectPythonLoggingToROS())
    # logs sent to children of trigger with a level >= this will be redirected
    # to ROS
    
    # logging.getLogger('server').setLevel(logging.DEBUG)

    try:
        node = GatewayNode()
    except Exception as e:
        rospy.logfatal("Caught exception: " + str(e))
    else:
        node.run()

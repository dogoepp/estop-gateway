#!/usr/bin/env python
# coding: utf-8

# Standard
import logging

# Ros-related
import rospy
from std_msgs.msg import UInt32

# Project imports
import server

class GatewayNode(object):
    def __init__(self, topic='pulse', queue_size=1):
        rospy.init_node('e_stop', anonymous=True, log_level=rospy.DEBUG)
        max_delay = 2 # seconds
        key = b"16:40:35"
        source_ip = '152.81.10.184'
        source_ip = '152.81.70.17'
        self.server = server.HeartBeatGateway(1042, max_delay, key, source_ip, 0.1)
        self.sliding_window = server.SlidingWindow(server.median, 50)

        self.publisher = rospy.Publisher(topic, UInt32, queue_size=queue_size)

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.process_message()
        except rospy.ROSInterruptException:
            pass

    def process_message(self):
        try:
            tick = self.server.receive_tick()

            # Check that we did receive data
            if tick["decoded"] is not None:
                rospy.logdebug("received tick")

                # Check the data's validity
                if self.server.check_data(tick):
                    battery_level = tick['decoded'][2]
                    # We have to rely on a smoothing method because the ADC
                    # (analog to digital converter) measurements are noisy, as
                    # discussed in https://github.com/esp8266/Arduino/issues/2070
                    battery_level = self.sliding_window(battery_level)

                    # self.relay_tick()
                    # self.send_battery_status()

                    rospy.loginfo("correct tick received")
                    rospy.loginfo("Battery level: {0}".format(battery_level))
                    rospy.loginfo("             : {0}".format(tick['decoded'][2]))
                else:
                    rospy.loginfo("The received tick is invalid.")
        except socket.error:
            pass

class ConnectPythonLoggingToROS(logging.Handler):
    """
    Class interfacing logs using Python's standard logging facility and ROS's
    way of logging. It is meant to be used as a Handler for the logging module.

    Source: https://gist.github.com/nzjrs/8712011
    """

    MAP = {
        logging.DEBUG:rospy.logdebug,
        logging.INFO:rospy.loginfo,
        logging.WARNING:rospy.logwarn,
        logging.ERROR:rospy.logerr,
        logging.CRITICAL:rospy.logfatal
    }

    def emit(self, record):
        try:
            self.MAP[record.levelno]("%s: %s" % (record.name, record.msg))
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno, record.name, record.msg))

if __name__ == '__main__':
    #reconnect logging calls which are children of this to the ros log system
    logging.getLogger('server').addHandler(ConnectPythonLoggingToROS())
    #logs sent to children of trigger with a level >= this will be redirected to ROS
    logging.getLogger('server').setLevel(logging.DEBUG)

    try:
        node = GatewayNode()
    except Exception as e:
        rospy.logfatal("Caught exception: " + str(e))
    else:
        node.run()

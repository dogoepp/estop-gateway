#!/usr/bin/env python
# coding: utf-8

import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # TODO: have a look to what
# can be done with `protocol`

host = socket.gethostname() # get local machine name
print("Host : " + str(host))
port = 1042 # the port we chose to listen to
s.bind(('', port)) # bind the socket to the port

while True:
    data, addr = s.recvfrom(1024) # buffer size is 1024 bytes
    print("received message : " + str(data))



# import rospy
# from std_msgs.msg import String
#
# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()
#
# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass
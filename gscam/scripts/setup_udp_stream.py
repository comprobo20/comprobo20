#!/usr/bin/env python3

import rospy
import socket
from os import system

rospy.init_node('setup_udp_stream')

receive_port = rospy.get_param('~receive_port')
host = rospy.get_param('~host')
width = rospy.get_param('~width')
height = rospy.get_param('~height')
fps = rospy.get_param('~fps')

video_mode = "-vf -ex sports -awb off -mm matrix -w " + str(width) + " -h " + str(height) + " -fps " + str(fps) + " -b 2000000"

port = 10003
size = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host,port))
video_mode_str = str(receive_port) + "," + str(video_mode) +"\n"
s.send(video_mode_str.encode())
all_data = ""
while not all_data.endswith('\n'):
	data = s.recv(size)
	all_data += data.decode('utf-8')
s.close()
system('hping3 -c 1 -2 -s ' + str(receive_port) + ' -p ' + all_data.strip() + ' ' + host)
print('Received:', all_data)

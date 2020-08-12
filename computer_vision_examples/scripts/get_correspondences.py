#!/usr/bin/env python3

import cv2
import pickle
import numpy as np
import sys
import rospkg

colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,255,255)]
scale_factor = 2.0
pt_num = 0
im1_pts = []
im2_pts = []

def mouse_event(event,x,y,flag,im):
	global im1_pts
	global im2_pts
	global pt_num

	if event == cv2.EVENT_FLAG_LBUTTON:
		cv2.circle(im,(x,y),2,colors[pt_num//2 % len(colors)],2)
		cv2.imshow("MYWIN",im)
		if x < im.shape[1]/2.0:
			im1_pts.append((x//scale_factor,y//scale_factor))
		else:
			im2_pts.append(((x - im.shape[1]/2.0)//scale_factor,y//scale_factor))
		pt_num += 1

if __name__ == '__main__':
	if len(sys.argv) < 2:
		print("USAGE: ./get_correspondences.py correspondence-file.pickle [num_correspondences]")
		sys.exit(1)
	if len(sys.argv) < 3:
		n_correspondences = 8
	else:
		n_correspondences = int(sys.argv[2])
	rospack = rospkg.RosPack()
	im1_file = rospack.get_path('computer_vision_examples') + '/images/frame0000.jpg'
	im2_file = rospack.get_path('computer_vision_examples') + '/images/frame0001.jpg'

	im1 = cv2.imread(im1_file)
	im2 = cv2.imread(im2_file)
	im1 = cv2.resize(im1,(int(im1.shape[1]*scale_factor),int(im1.shape[0]*scale_factor)))
	im2 = cv2.resize(im2,(int(im2.shape[1]*scale_factor),int(im2.shape[0]*scale_factor)))

	im = np.array(np.hstack((im1,im2)))
	cv2.imshow("MYWIN",im)
	cv2.setMouseCallback("MYWIN",mouse_event,im)
	while True:
		if len(im2_pts) >= n_correspondences:
			break
		cv2.waitKey(50)

	f = open(sys.argv[1],'wb')
	pickle.dump((im1_pts,im2_pts),f)
	f.close()
	cv2.destroyAllWindows()

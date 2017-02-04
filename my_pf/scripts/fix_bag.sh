#!/bin/sh

rosbag filter $1 $2 "topic != 'tf' or m.transforms[0].child_frame_id not in ['odom', 'STAR']"

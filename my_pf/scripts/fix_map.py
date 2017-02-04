#!/usr/bin/env python

import yaml
import sys
from math import pi
import cv2
import numpy as np

if len(sys.argv) < 2:
    print "USAGE: ./fix_map.py path-to-YAML-file"
    sys.exit(1)

if not sys.argv[1].endswith('.yaml'):
    print "invalid yaml file.  Must end in .yaml"
    sys.exit(1)

yaml_file = sys.argv[1]
yaml_file_fixed = sys.argv[1][:-5] + '_unrotated.yaml'

# read the YAML file
with open(yaml_file, 'r') as stream:
    try:
        map_yaml = yaml.load(stream)
    except Exception as inst:
        print "Error", inst
        sys.exit(1)

last_slash = yaml_file.rfind('/')
if last_slash == -1:
    image_path = map_yaml['image']
else:
    image_path = yaml_file[:(last_slash+1)] + map_yaml['image']
image_path_fixed = map_yaml['image'][:-4] + '_unrotated.pgm'

# rotate the image about the center pixel
I = cv2.imread(image_path)
rows, cols, _ = I.shape
M = cv2.getRotationMatrix2D((cols/2,rows/2),180.0/pi*map_yaml['origin'][2],1)
# Note: the value for unknown obstacle status is hardcoded to 205
rotated = cv2.warpAffine(I,M,(cols,rows), borderValue=(205, 205, 205))

# compute the map coordinates of the center pixel
rx, ry = cols/2.0*map_yaml['resolution'], rows/2.0*map_yaml['resolution']
rotated_origin = np.linalg.inv(M[:,:-1]).dot([rx, ry]) + np.asarray([map_yaml['origin'][0],
                                                                     map_yaml['origin'][1]])

# figure out the map coordinates of the new pixel origin
rotated_rows, rotated_cols, _ = rotated.shape
map_origin = [rotated_origin[0] - rotated_cols/2.0*map_yaml['resolution'],
              rotated_origin[1] - rotated_rows/2.0*map_yaml['resolution']]
cv2.imwrite(image_path_fixed, rotated)

# update the YAML to reflect the new PGM and the new origin
map_yaml['image'] = image_path_fixed
map_yaml['origin'] = [float(map_origin[0]), float(map_origin[1]), 0.0]

# write the YAML to disk
with open(yaml_file_fixed, 'w') as stream:
    yaml.dump(map_yaml, stream)

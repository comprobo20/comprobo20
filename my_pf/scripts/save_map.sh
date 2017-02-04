#!/bin/bash

rosrun map_server map_saver -f $1 map:=transformed_map
rosrun my_pf fix_map.py $1.yaml

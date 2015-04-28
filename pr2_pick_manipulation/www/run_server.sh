#!/bin/bash

roslaunch pr2_pick_manipulation ee_reader.launch &

# Serve index.html on port 8080 so you can see the result on your phone or laptop.
roscd pr2_pick_manipulation/www
python -m SimpleHTTPServer 8080 .

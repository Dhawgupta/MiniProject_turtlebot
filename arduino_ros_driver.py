#!/usr/bin/env python

''' this node will monitor the sensor and speed values  for tha arduino


 [Arduino] ----odom---> [this node] --->
             <--speed---            <-----
'''
import rospy
import sys
import time
import math

from std_msgs.msg import Int16, Int32,Int64,Float32, String, UInt64, Header




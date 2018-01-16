#! /usr/bin/env python

import rospy
import time
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from ardrone_autonomy.msg import Navdata  # message type for nav_callback
from control_lab.msg import EstimatedState
import VisualOdometry

from Queue import PriorityQueue


class Coprocessor:
    def __init__(self):
        visual_odometry = VisualOdometry(channel=0,
                                         height=240,
                                         width=320,
                                         scale=0.1,
                                         reset=1,
                                         min=3,
                                         max=4,
                                         eigen=0.05,
                                         debug=False):

    def run(self):

if __name__ == '__main__':
    estimator = Estimator(Filter())
    print 'Estimation Running'
    estimator.run()
# API Training Shop Blog About


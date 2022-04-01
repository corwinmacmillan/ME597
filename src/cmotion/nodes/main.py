#!/usr/bin/env python3

from Q2 import *
from mround import *
import rospy

if __name__ == "__main__":
    try:
        Q2()
    except rospy.ROSInterruptException:
        pass

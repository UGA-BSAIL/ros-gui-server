#!/usr/bin/env python

from ros_gui_server.srv import OdomConversion, OdomConversionResponse
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

def handle_odom_conversion(fix):
    odom = Odometry()
    return OdomConversionResponse(odom)


def odom_conversion_server():
    rospy.init_node('odom_conversion_server')
    s = rospy.Service('odom_conversion', OdomConversion, handle_odom_conversion)
    print "Ready for Odometry Conversion"
    rospy.spin()

if __name__ == "__main__":
    odom_conversion_server()
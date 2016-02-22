#!/usr/bin/env python
# need ros-jade-marti-visualization-msgs and ros-jade-mapviz-plugins? 
# This doesn't work for rviz, only mapviz has support for displaying this

import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from marti_visualization_msgs.msg import TexturedMarker


rospy.init_node('textured_marker')

bridge = CvBridge()
image_name = rospy.get_param("~image_name")
cv_image = cv2.imread(image_name)
image = bridge.cv2_to_imgmsg(cv_image, "bgr8")

pub = rospy.Publisher("textured_marker", TexturedMarker, queue_size=1)

tm = TexturedMarker()

tm.header.stamp = rospy.Time.now()
tm.header.frame_id = "map"
tm.image = image
# TODO(lucasw) does image header matter?
tm.image.header = tm.header
tm.pose.orientation.w = 1.0
tm.alpha = 0.8
tm.resolution = 0.01
tm.ns = "test"
tm.id = 0

while not rospy.is_shutdown():
    pub.publish(tm)

    rospy.sleep(1.0)

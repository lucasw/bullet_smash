#!/usr/bin/env python
# create a unit that starts at the right of the grid and advances at a
# given speed.

# import roslib

import rospy
import tf

from visualization_msgs.msg import Marker

class Enemy:
    def __init__(self):
        self.br = tf.TransformBroadcaster()

        name = rospy.get_name()
        namespace = rospy.get_namespace()

        self.link = name.replace(namespace, "", 1)
        rospy.loginfo("link \"" + self.link + "\"")

        self.marker = Marker()
        self.marker.header.frame_id = self.link
        self.marker.type = self.marker.CUBE
        self.marker.scale.x = 0.8
        self.marker.scale.y = 0.8
        self.marker.scale.z = 0.8
        self.marker.color.a = 1.0
        self.marker_pub = rospy.Publisher("marker", Marker, queue_size = 1)

        # how fast the unit advances
        self.seconds_per_square = rospy.get_param("seconds_per_square", 6)
        self.vel = -1.0 / self.seconds_per_square

        self.init_x = rospy.get_param("~x", 10)
        self.x = self.init_x
        self.init_y = rospy.get_param("~y", 1)
        self.y = self.init_y
        self.z = 0

        self.dt = 0.05
        rospy.Timer(rospy.Duration(self.dt), self.update)

    def update(self, event):
        self.x += self.vel * self.dt
        if self.x < 0:
           self.x = self.init_x
        self.br.sendTransform((self.x, self.y, self.z),
                              (0, 0, 0, 1.0),
                              rospy.Time.now(),
                              self.link,
                              "map")
        # TODO(lucasw) Do I need to keep publishing this over and over?
        self.marker_pub.publish(self.marker)
if __name__ == '__main__':
    rospy.init_node('enemy')
    enemy = Enemy()
    rospy.spin()

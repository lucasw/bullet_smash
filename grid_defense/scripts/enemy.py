#!/usr/bin/env python
# create a unit that starts at the right of the grid and advances at a
# given speed.

# import roslib

import rospy
import tf

from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class Enemy:
    def __init__(self):
        self.br = tf.TransformBroadcaster()

        self.name = rospy.get_name()
        namespace = rospy.get_namespace()

        self.link = self.name.replace(namespace, "", 1)
        rospy.loginfo("link \"" + self.link + "\"")

        self.marker_array = MarkerArray()

        # base black cube
        marker = Marker()
        marker.header.frame_id = self.link
        marker.type = marker.CUBE
        marker.id = 0
        marker.scale.x = 0.7
        marker.scale.y = 0.7
        marker.scale.z = 0.7
        marker.color.a = 1.0
        self.marker_array.markers.append(marker)

        # white on black
        marker = Marker()
        marker.header.frame_id = self.link
        marker.type = marker.CUBE
        marker.id = 1
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 255
        marker.color.g = 255
        marker.color.b = 255
        marker.pose.position.z = 1.0
        self.marker_array.markers.append(marker)

        self.marker_pub = rospy.Publisher("marker_array", MarkerArray, queue_size=1)

        # how fast the unit advances
        self.seconds_per_square = rospy.get_param("seconds_per_square", 6)
        self.vel = -1.0 / self.seconds_per_square

        self.init_x = rospy.get_param("~x", 10)
        self.x = self.init_x
        self.init_y = rospy.get_param("~y", 1)
        self.y = self.init_y
        self.z = 0

        self.dt = 0.05

        self.hit_points = 100.0

        self.update_timer = rospy.Timer(rospy.Duration(self.dt), self.update)

        self.damage_sub = rospy.Subscriber(self.name + "/damage",
                                           Float32, self.damage, queue_size=1)

    def damage(self, msg):
        self.hit_points -= msg.data
        rospy.loginfo("hp " + str(self.hit_points) + " " + str(msg.data))

        if self.hit_points < 0:
            msg = "this unit is dead " + str(self.hit_points)
            rospy.loginfo(msg)
            rospy.signal_shutdown(msg)

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
        self.marker_pub.publish(self.marker_array)


if __name__ == '__main__':
    rospy.init_node('enemy')
    enemy = Enemy()
    # while not rospy.is_shutdown():
    #    rospy.sleep(0.5)
    rospy.spin()

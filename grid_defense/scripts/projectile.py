#!/usr/bin/env python
# create a unit that starts at the right of the grid and advances at a
# given speed.

# import roslib

import rospy
import tf

from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


# TODO(lucasw) make this inherit from a base class
class Projectile:
    def __init__(self):
        self.br = tf.TransformBroadcaster()

        self.name = rospy.get_name()
        namespace = rospy.get_namespace()

        self.link = self.name.replace(namespace, "", 1)
        rospy.loginfo("link \"" + self.link + "\"")

        self.marker_array = MarkerArray()

        scale = 0.2
        # TODO(lucasw) make these a mesh instead
        # base black cube
        marker = Marker()
        marker.header.frame_id = self.link
        marker.type = marker.CUBE
        marker.id = 0
        marker.ns = self.name
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.frame_locked = True
        # self.marker_array.markers.append(marker)

        # white on black
        marker2 = Marker()
        marker2.header.frame_id = self.link
        marker2.type = marker.CUBE
        marker2.id = 1
        marker2.ns = self.name
        marker2.scale.x = scale - 0.1
        marker2.scale.y = scale - 0.1
        marker2.scale.z = 0.1
        marker2.color.a = 1.0
        marker2.color.r = 255
        marker2.color.g = 255
        marker2.color.b = 255
        marker2.pose.position.z = 1.0
        marker2.frame_locked = True
        # self.marker_array.markers.append(marker2)

        # energy
        self.energy_bar_orig_length = scale - 0.1
        self.energy_bar = Marker()
        self.energy_bar.header.frame_id = self.link
        self.energy_bar.type = self.energy_bar.CUBE
        self.energy_bar.id = 2
        self.energy_bar.ns = self.name
        self.energy_bar.scale.x = self.energy_bar_orig_length
        self.energy_bar.scale.y = 0.1
        self.energy_bar.scale.z = 0.1
        self.energy_bar.color.a = 1.0
        self.energy_bar.color.r = 0.05
        self.energy_bar.color.g = 1.0
        self.energy_bar.color.b = 0.05
        self.energy_bar.pose.position.z = 1.1
        self.energy_bar.pose.position.y = -0.2
        self.energy_bar.frame_locked = True

        # self.marker_pub = rospy.Publisher("/marker_array", MarkerArray, queue_size=1)
        self.marker_pub = rospy.Publisher("/marker", Marker, queue_size=4)

        # how fast the unit advances
        self.seconds_per_square = rospy.get_param("seconds_per_square", 3)
        self.vel = 1.0 / self.seconds_per_square

        self.init_x = rospy.get_param("~x", 0)
        self.x = self.init_x
        self.init_y = rospy.get_param("~y", 1)
        self.y = self.init_y
        self.z = 0

        self.dt = 0.05

        self.hit_points_orig = 10.0
        self.hit_points = self.hit_points_orig

        self.update_timer = rospy.Timer(rospy.Duration(self.dt), self.update)

        self.damage_sub = rospy.Subscriber(self.name + "/damage",
                                           Float32, self.damage, queue_size=1)

        rospy.sleep(0.5)
        # self.marker_pub.publish(self.marker_array)
        self.marker_pub.publish(marker)
        self.marker_pub.publish(marker2)
        self.marker_pub.publish(self.energy_bar)

    def damage(self, msg):
        self.hit_points -= msg.data
        rospy.loginfo("hp " + str(self.hit_points) + " " + str(msg.data))

        self.energy_bar.scale.x = self.hit_points / self.hit_points_orig * \
                self.energy_bar_orig_length
        self.energy_bar.pose.position.x = -self.energy_bar_orig_length / 2.0 + \
                self.energy_bar.scale.x / 2.0
        self.marker_pub.publish(self.energy_bar)

        if self.hit_points < 0:
            msg = "this unit is dead " + str(self.hit_points)
            rospy.loginfo(msg)
            rospy.signal_shutdown(msg)

    def update(self, event):
        self.x += self.vel * self.dt
        self.br.sendTransform((self.x, self.y, self.z),
                              (0, 0, 0, 1.0),
                              rospy.Time.now(),
                              self.link,
                              "map")
        if self.x > 10:
            msg = "this unit is off screen"
            rospy.loginfo(msg)
            rospy.signal_shutdown(msg)

if __name__ == '__main__':
    rospy.init_node('projectile')
    projectile = Projectile()
    # while not rospy.is_shutdown():
    #    rospy.sleep(0.5)
    rospy.spin()

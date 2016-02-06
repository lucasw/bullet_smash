#!/usr/bin/env python
# create a unit that starts at the right of the grid and advances at a
# given speed.

# import roslib

import rospy
import tf

class Enemy:
    def __init__(self):
        self.br = tf.TransformBroadcaster()

        name = rospy.get_name()
        namespace = rospy.get_namespace()

        self.link = name.replace(namespace, "", 1)
        rospy.loginfo("link \"" + self.link + "\"")

        # how fast the unit advances
        self.seconds_per_square = rospy.get_param("seconds_per_square", 6)
        self.vel = -1.0 / self.seconds_per_square

        self.x = rospy.get_param("~x", 10)
        self.y = rospy.get_param("~y", 1)
        self.z = 0

        self.dt = 0.05
        rospy.Timer(rospy.Duration(self.dt), self.update)

    def update(self, event):
        self.x += self.vel * self.dt
        self.br.sendTransform((self.x, self.y, self.z),
                              (0, 0, 0, 1.0),
                              rospy.Time.now(),
                              self.link,
                              "map")
if __name__ == '__main__':
    rospy.init_node('enemy')
    enemy = Enemy()
    rospy.spin()

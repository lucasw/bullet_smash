#!/usr/bin/env python
# Copyright 2016 Lucas Walter
#
# Act as a simulator to manage game state, publish a clock
# publish tf in response to movements attempted by actor/robots
# publish sensor information that actors request-
# the sensor information reveals a subset of game state to the
# actor, e.g. there are obstacles around, there are allies/enemies
# at certain relative positions- the game manager would have
# determine line of sight.
# Manage weapons- if an actor fires a projectile, determine if it
# hits an actor and what the response is.

import rospy
import tf
import time

# from twisted.internet import task, reactor
from rosgraph_msgs.msg import Clock


class GameManager():
    def __init__(self):
        rospy.init_node('game_manager')
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
        self.clock = Clock()
        self.dt = 0.1
        self.nsecs_per_sec = 1e9
        # TODO(lucasw) have ability to change rate, pause
        self.rate = 1.0
        while not rospy.is_shutdown():
            self.update()
            time.sleep(self.dt)
        # self.loop = task.LoopingCall(self.update)
        # self.loop.start(self.dt)
        # reactor.run()

    # since the clock isn't running, can't depend on ros to trigger a timer update
    # to itself generate a clock.
    def update(self):
        # if rospy.is_shutdown():
        #     self.loop.stop()
        self.clock.clock.nsecs += self.dt * self.nsecs_per_sec
        if self.clock.clock.nsecs > self.nsecs_per_sec:
            self.clock.clock.nsecs -= self.nsecs_per_sec
            self.clock.clock.secs += 1

        self.clock_pub.publish(self.clock)

if __name__ == '__main__':
   game_manager = GameManager()
   rospy.spin()

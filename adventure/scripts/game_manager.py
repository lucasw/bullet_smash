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

# Need a rosservice where an actor node can register itself, provide the name of 
# the Point2D command input, the tf link name, and later footprint of the
# actor.
# or at least just provide those in yaml form at launch time.

import rospy
import tf
import time

from opencv_apps.msg import Point2D
# from twisted.internet import task, reactor
from rosgraph_msgs.msg import Clock

class GameManager():
    def __init__(self):
        # later need a list of topics to subscribe to
        # TODO(lucasw) need an entire Actor class to put into dict
        # it would have the topic names, the link names, and current
        # position in it
        self.move_topics = {}
        self.move_topics['player'] = '/player/cmd_xy'

        self.br = tf.TransformBroadcaster()

        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
        self.clock = Clock()
        self.dt = 0.1
        self.nsecs_per_sec = 1e9
        # TODO(lucasw) have ability to change rate, pause
        self.rate = 1.0

        self.subs = {}
        self.moves = {}
        for key in self.move_topics:
            self.subs[key] = rospy.Subscriber(key + "/cmd_xy", Point2D,
                                              self.cmd_xy_callback, key,
                                              queue_size=1)
            self.moves[key] = Point2D()

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

        now = rospy.Time()
        now.secs = self.clock.clock.secs
        now.nsecs = self.clock.clock.nsecs

        for actor in self.moves:
            pos = self.moves[actor]
            self.br.sendTransform((pos.x, pos.y, 0.0),
                                  (0, 0, 0, 1.0),
                                  now,
                                  actor,
                                  "map")

    def cmd_xy_callback(self, msg, actor):
        # TODO(lucasw) later make this so the msg is only added
        # at update time, so if two arrive only one is used
        # Later need to check move against map
        self.moves[actor].x += msg.x
        self.moves[actor].y += msg.y

if __name__ == '__main__':
    rospy.init_node('game_manager')
    game_manager = GameManager()
    rospy.spin()

#!/usr/bin/env python

import rospy

from keyboard.msg import Key
from opencv_apps.msg import Point2D

class Player():
    def __init__(self):
        self.dx = 0
        self.dy = 0
        self.up = False
        self.up_pressed = False
        self.down = False
        self.down_pressed = False

        self.move_pub = rospy.Publisher("cmd_xy", Point2D, queue_size=1)
        self.key_sub = rospy.Subscriber("/keyboard/keydown", Key, self.keypress,
                                        True,
                                        queue_size=4)
        self.key_sub = rospy.Subscriber("/keyboard/keyup", Key, self.keypress,
                                        False,
                                        queue_size=4)
        rospy.Timer(rospy.Duration(0.2), self.update)

    def update(self, event):
        if self.up and not self.down:
            self.dy = 1.0
        elif not self.up and self.down:
            self.dy = -1.0
        # momentary events
        elif self.up_pressed and not self.down_pressed:
            self.dy = 1.0
        elif not self.up_pressed and self.down_pressed:
            self.dy = -1.0
        else:
            self.dy = 0.0

        cmd_xy = Point2D()
        cmd_xy.x = self.dx
        cmd_xy.y = self.dy
        self.move_pub.publish(cmd_xy)

        self.up_pressed = False
        self.down_pressed = False

    # Want to capture both momentary pressed and continual presses
    # The continual presses should continue to generate movements
    # in the desired direction in update.
    def keypress(self, msg, press_not_release):
        if press_not_release:
            if msg.code == Key.KEY_UP:
                if not self.up:
                    self.up_pressed = True
                self.up = True
            if msg.code == Key.KEY_DOWN:
                if not self.down:
                    self.down_pressed = True
                self.down = True
        else:
            if msg.code == Key.KEY_UP:
                self.up = False
            if msg.code == Key.KEY_DOWN:
                self.down = False


if __name__ == '__main__':
    rospy.init_node("player")
    player = Player()
    rospy.spin()

#!/usr/bin/env python

import roslaunch
import rospy


class EnemyManager:
    def __init__(self):
        self.ns = rospy.get_namespace()

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        names = ['t1', 't2', 't3']
        self.nodes = {}
        self.processes = {}
        y = 0
        for name in names:
            y += 1
            param_name = "/" + name + "/y"
            if self.ns != "/":
                param_name = self.ns + param_name
            rospy.set_param(param_name, y)
            self.nodes[name] = roslaunch.core.Node("grid_defense", "enemy.py",
                                                   name=name, namespace=self.ns,
                                                   args="")
            self.processes[name] = self.launch.launch(self.nodes[name])

        # while not rospy.is_shutdown():
        #    rospy.sleep(1.0)
        rospy.spin()

        for name in names:
            rospy.loginfo("stopping " + name)
            self.processes[name].stop()

if __name__ == '__main__':
    enemy_manager = EnemyManager()

#!/usr/bin/env python

from RRbotController import RRbotController
import rospy


rrc = RRbotController()
rrc.move_joint([0, 0], duration=2.0)

for i in range(20):
    jnt_cmd = [0.5 * i, 0.75 * i]
    bag_fname = 'rrbot-gc-%02d.bag' % (i)
    rrc.move_joint_and_log(jnt_cmd, bag_fname)
    rospy.loginfo('{}: Moved rrbot to {}'.format(i, jnt_cmd))

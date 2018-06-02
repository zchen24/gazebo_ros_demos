#!/usr/bin/env python


from datetime import datetime
import numpy as np
import rospy
from PyKDL import *
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rosbag
import roslaunch


class RRbotController(object):

    NUM_JNTS = 2

    def __init__(self, record=False):
        rospy.init_node('gc_data_collector')
        rospy.loginfo('RRbot Controller Created')

        self.sub = rospy.Subscriber('/rrbot/joint_states',
                                    JointState, self.callback_joint_states)

        self.pub_j1 = rospy.Publisher('/rrbot/joint1_position_controller/command',
                                      Float64, queue_size=1)
        self.pub_j2 = rospy.Publisher('/rrbot/joint2_position_controller/command',
                                      Float64, queue_size=1)
        self.jnt_pubs = [self.pub_j1, self.pub_j2]

        self.jnt_cmd = JntArray(2)
        now = datetime.now()

        if record:
            self.bag = rosbag.Bag(now.strftime('%Y-%m-%d-%H-%M-gravity.bag'), 'w')
        else:
            self.bag = None
        self.state = None
        self.ps_bag = None

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

    def __del__(self):
        self.close()

    def close(self):
        rospy.signal_shutdown('')
        if self.bag is not None:
            self.bag.close()

    def run(self):
        pass

    def move_joint(self, jnt_cmd, duration = None):
        """
        :param jnt_cmd: target joint positions
        :param blocking: whether the moving is blocking
        """
        if len(jnt_cmd) != self.NUM_JNTS:
            rospy.logerr('joint command length error')
            return

        if duration is None:
            for i, cmd in enumerate(jnt_cmd):
                self.jnt_pubs[i].publish(cmd)
            return

        # publish
        period = 0.005   # 5 ms
        num_steps = int(duration / period)
        start_pos = self.state.position
        jnt_trajectories = []

        for i, cmd in enumerate(jnt_cmd):
            jnt_trajectories.append(np.linspace(start_pos[i], cmd, num_steps))

        for i in range(num_steps):
            for j in range(self.NUM_JNTS):
                self.jnt_pubs[j].publish(jnt_trajectories[j][i])
                rospy.sleep(period)

    def move_joint_and_log(self, jnt_cmd, log_name="tmp.bag"):

        # start bag
        bag_node = roslaunch.core.Node('rosbag', 'record',
                                       args='-a -O {}'.format(log_name))
        # bag_node = roslaunch.core.Node('rosbag', 'record',
        #                                args='-a', output='screen')


        self.ps_bag = self.launch.launch(bag_node)
        self.move_joint(jnt_cmd)

        rospy.sleep(5.0)
        self.ps_bag.stop()

    def callback_joint_states(self, msg):
        self.state = msg
        if self.bag is not None and self.bag:
            self.bag.write("/rrbot/joint_states", msg)


if __name__ == '__main__':
    rospy.loginfo('Data collector started')

    controller = RRbotController()


    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        controller.run()




#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

class Teleop:
    def __init__(self):
        # create node
        if not rospy.get_node_uri():
            rospy.init_node('teleop_node', anonymous=True, log_level=rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

        # To Get Gripper States
        self.m = []     # "m" denotes master device (i.e. Geomagic Touch)
        self.s = []     # "s" denotes slave device (i.e. Panda robot arm)
        self._joint_names_geomagic = ['m1', 'm2', 'm3', 'm4', 'm5', 'm6']
        self._joint_names_panda = ['s1', 's2', 's3', 's4', 's5', 's6', 's7']
        self._joint_states_state_sub = rospy.Subscriber('/omni/joint_states', JointState, self._joint_states_callback, queue_size=1, tcp_nodelay=True)
        self._set_joint_pub = rospy.Publisher('/panda/joint_states', JointState, latch=True, queue_size=1)

        rospy.sleep(1)
        while not rospy.is_shutdown():
            self.s = self.mapping(self.m)
            self.s = self.s + [0.0]
            self.set_joint_position(self.s)
            rospy.sleep(0.01)


    """
    Get Joint Positions
    """
    def _joint_states_callback(self, msg):
        if msg.name == self._joint_names_geomagic:
            self.m = list(msg.position)
        elif msg.name == self._joint_names_panda:
            self.s = list(msg.position)

    """
    Set Joint Positions
    """
    def set_joint_position(self, s):     # Set Joint Angles of Panda robot
        msg = JointState()
        msg.name = self._joint_names_panda
        msg.position = s
        self._set_joint_pub.publish(msg)

    """
    Master-Slave Mapping
    """
    def mapping(cls, m):
        s = m
        return s


if __name__ == "__main__":
    tel = Teleop()

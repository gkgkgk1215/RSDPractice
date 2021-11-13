#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from panda.teleop.omniKinematics import omniKinematics
from panda.pandaKinematics import pandaKinematics


class pandaROS:
    def __init__(self):
        # create node
        if not rospy.get_node_uri():
            rospy.init_node('panda_ros_node', anonymous=True, log_level=rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

        # member variables
        self.joints = []
        self._joint_names_panda = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                                   'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']

        # ROS publisher
        self._set_joint_pub = rospy.Publisher('/panda/joint_states', JointState, latch=True, queue_size=1)

        # ROS subscriber
        sub_topic = '/joint_states'
        rospy.Subscriber(sub_topic, JointState, self.joint_states_callback, queue_size=1)
        rospy.wait_for_message(topic=sub_topic, topic_type=JointState)



    """
    Get Joint Positions
    """
    def joint_states_callback(self, msg):
        self.joints = list(msg.position)    # exclude both finger joints

    """
    Set Joint Positions
    """
    def set_joint_position(self, s):     # Set Joint Angles of Panda robot
        msg = JointState()
        msg.name = self._joint_names_panda
        msg.position = s
        self._set_joint_pub.publish(msg)


if __name__ == "__main__":
    panda = pandaROS()
    joints = [0.0, -0.702, -0.678, -2.238, -0.365, 0.703, 0.808, 0.0, 0.0]

    # Test: Simple sinusoidal move of the first joint
    import time
    t = 0.0
    while True:
        joints[0] = 1.5*np.sin(2*np.pi*t/10)
        panda.set_joint_position(joints)
        t += 0.01
        time.sleep(0.01)
        print (panda.joints)
#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from omniKinematics import omniKinematics
from pandaKinematics import pandaKinematics


class teleop:
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
        self._joint_names_panda = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                                   'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']
        rospy.Subscriber('/omni/joint_states', JointState, self._omni_joint_states_callback, queue_size=1)
        rospy.Subscriber('/panda/joint_states', JointState, self._panda_joint_states_callback, queue_size=1)
        self._set_joint_pub = rospy.Publisher('/teleop/panda/joint_states', JointState, latch=True, queue_size=1)

        while not rospy.is_shutdown():
            if self.m == [] or self.s == []:
                pass
            else:
                s_new = self.mapping(self.m, self.s)
                self.set_joint_position(np.concatenate((s_new, [0.0, 0.0])))


    """
    Get Joint Positions
    """
    def _omni_joint_states_callback(self, msg):
        self.m = list(msg.position)

    def _panda_joint_states_callback(self, msg):
        self.s = list(msg.position)    # exclude both finger joints

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
    @classmethod
    def mapping(cls, m, s):
        """
        m : Current 6 joint angles of master manipulator (i.e. Geomagic Touch)
        s : Current 7 joint angles of slave manipulator (i.e. Panda)
        return: joint angles of
        """
        # Get the fk from master
        Tm = omniKinematics.fk(joints=m)

        # Motion scale & offset pos
        pos = Tm[:3, -1]
        pos *= 5.0
        pos += [0.5, 0.8, 0.3]
        Tm[:3, -1] = pos

        # Solve the ik and set the joint angles of panda
        s_new = pandaKinematics.ik(Tb_ed=Tm, q0=s[:7], RRMC=True)
        s_new_null = pandaKinematics.null_space_control(joints=s[:7], crit='joint_limit')
        return s_new + s_new_null


if __name__ == "__main__":
    tel = teleop()

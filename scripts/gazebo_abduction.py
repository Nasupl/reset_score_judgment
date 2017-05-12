#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist

import math


class GazeboAbduction:
    def __init__(self):
        rospy.init_node('gazebo_abduction')
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_cb)

        abduction_pose_cfg = rospy.get_param('~abduction_pose')
        self.abduction_pose = Pose()
        self.abduction_pose.position.x = abduction_pose_cfg['position']['x']
        self.abduction_pose.position.y = abduction_pose_cfg['position']['y']
        self.abduction_pose.position.z = abduction_pose_cfg['position']['z']
        self.abduction_pose.orientation.x = abduction_pose_cfg['orientation']['x']
        self.abduction_pose.orientation.y = abduction_pose_cfg['orientation']['y']
        self.abduction_pose.orientation.z = abduction_pose_cfg['orientation']['z']
        self.abduction_pose.orientation.w = abduction_pose_cfg['orientation']['w']

        target_pose_cfg = rospy.get_param('~target_pose')
        self.target_pose = Pose()
        self.target_pose.position.x = target_pose_cfg['position']['x']
        self.target_pose.position.y = target_pose_cfg['position']['y']
        self.target_pose.position.z = target_pose_cfg['position']['z']
        self.target_pose.orientation.x = target_pose_cfg['orientation']['x']
        self.target_pose.orientation.y = target_pose_cfg['orientation']['y']
        self.target_pose.orientation.z = target_pose_cfg['orientation']['z']
        self.target_pose.orientation.w = target_pose_cfg['orientation']['w']

    def calc_dist(self, pos_a, pos_b):
        print pos_a.position.x
        print pos_b.position.x
        if not (hasattr(pos_a.position, 'x') and hasattr(pos_b.position, 'x')):
            raise TypeError
        if not (hasattr(pos_a.position, 'y') and hasattr(pos_b.position, 'y')):
            raise TypeError
        return math.sqrt(math.pow(pos_a.position.x - pos_b.position.x, 2) + math.pow(pos_a.position.y - pos_b.position.y, 2))

    def get_abduction_dist(self, now_pose):
        return self.calc_dist(self.abduction_pose, now_pose)

    def state_cb(self, data):
        target_index = data.name.index('icart_mini')
        now_pose = data.pose[target_index]
        # rospy.loginfo(self.get_abduction_dist(now_pose))
        if self.get_abduction_dist(now_pose) < 1.0:
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                ret = set_model_state(ModelState('icart_mini', self.target_pose, Twist(), ''))
            except rospy.ServiceException, e:
                rospy.logerr(e)

    def spin(self):
        rospy.loginfo('test')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo('test')
            rate.sleep()


if __name__ == '__main__':
    abduction = GazeboAbduction()
    abduction.spin()

#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Twist

import math
# import ctypes
import numpy as np
import numpy.random as rd


class GazeboAbduction:
    def __init__(self):
        rospy.init_node('gazebo_abduction')
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_cb)
        rospy.Subscriber('/costmap', OccupancyGrid, self.map_cb)
        self.pub = rospy.Publisher('/abduction', Bool)
        rospy.Timer(rospy.Duration(30), self.timer_cb)

        # abduction_pose_cfg = rospy.get_param('~abduction_pose')
        # self.abduction_pose.position.x = abduction_pose_cfg['position']['x']
        # self.abduction_pose.position.y = abduction_pose_cfg['position']['y']
        # self.abduction_pose.position.z = abduction_pose_cfg['position']['z']
        # self.abduction_pose.orientation.x = abduction_pose_cfg['orientation']['x']
        # self.abduction_pose.orientation.y = abduction_pose_cfg['orientation']['y']
        # self.abduction_pose.orientation.z = abduction_pose_cfg['orientation']['z']
        # self.abduction_pose.orientation.w = abduction_pose_cfg['orientation']['w']

        # target_pose_cfg = rospy.get_param('~target_pose')
        # self.target_pose.position.x = target_pose_cfg['position']['x']
        # self.target_pose.position.y = target_pose_cfg['position']['y']
        # self.target_pose.position.z = target_pose_cfg['position']['z']
        # self.target_pose.orientation.x = target_pose_cfg['orientation']['x']
        # self.target_pose.orientation.y = target_pose_cfg['orientation']['y']
        # self.target_pose.orientation.z = target_pose_cfg['orientation']['z']
        # self.target_pose.orientation.w = target_pose_cfg['orientation']['w']

        self.map = OccupancyGrid()
        self.now_pos = Point()
        self.now_pose = Pose()

        rospy.spin()

    def calc_dist(self, pos_a, pos_b):
        if not (hasattr(pos_a, 'x') and hasattr(pos_b, 'x')):
            raise TypeError
        if not (hasattr(pos_a, 'y') and hasattr(pos_b, 'y')):
            raise TypeError
        return math.sqrt(math.pow(pos_a.x - pos_b.x, 2) + math.pow(pos_a.y - pos_b.y, 2))

    def get_abduction_dist(self, now_pose):
        return self.calc_dist(self.target_pose, now_pose)

    def state_cb(self, data):
        target_index = data.name.index('icart_mini')
        self.now_pose = data.pose[target_index]
        # print self.now_pose
        # if self.get_abduction_dist(now_pose) < 1.0:
        #     rospy.wait_for_service('/gazebo/set_model_state')
        #     try:
        #         set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        #         ret = set_model_state('icart_mini', self.target_pose)
        #     except rospy.ServiceException, e:
        #         rospy.logerr(e)

    def map_cb(self, data):
        rospy.loginfo("receive map")
        self.map = data

    def get_pos_in_map_grid(self, pos):
        grid_x = int((pos.x - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = int((pos.y - self.map.info.origin.position.y) / self.map.info.resolution)
        # print self.map.info.width * (grid_y - 1) + grid_x
        return self.map.info.width * (grid_y - 1) + grid_x

    def is_grid_free(self, grid):
        return self.map.data[grid]

    def timer_cb(self, event):
        rospy.loginfo("timer_called")
        if not len(self.map.data):
            return
        is_in_free = False
        while not is_in_free:
            rand_cof = rd.rand(1, 2)
            rospy.loginfo(rand_cof)
            abduction_pos = Point(
                self.now_pose.position.x + 3 * rand_cof[0][0] * rd.choice([-1,1]), 
                self.now_pose.position.y + 3 * rand_cof[0][1] * rd.choice([-1,1]), 
                0)
            #  + rand_cof[0][1] * rd.choice([-1,1])
            abduction_grid = self.get_pos_in_map_grid(abduction_pos)
            if self.is_grid_free(abduction_grid) < 1:
                break
        abduction_pose = self.now_pose
        abduction_pose.position.x = abduction_pos.x
        abduction_pose.position.y = abduction_pos.y
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ret = set_model_state(ModelState('icart_mini', abduction_pose, Twist(), ""))
            self.pub.publish(Bool(True))
        except rospy.ServiceException, e:
            rospy.logerr(e)


if __name__ == '__main__':
    abdiction = GazeboAbduction()

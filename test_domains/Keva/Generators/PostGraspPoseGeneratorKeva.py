from openravepy import *
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import math
import random
from trac_ik_python.trac_ik import IK
import Config

class PostGraspPoseGeneratorKeva(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'obj', 'region']

        super(PostGraspPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('obj')
        self.grasp_region = known_argument_values.get('region')
        self.grasp_pose = known_argument_values.get('grasp_pose')
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_pose_list(self):
        pre_grasp_offset = 0.03
        pose_list =[]
        env = self.simulator.env
        robot = env.GetRobot(Config.ROBOT_NAME)
        pre_grasp_matrix = np.identity(4)
        pre_grasp_matrix[2][3] -= pre_grasp_offset

        final_pose = np.matmul(self.grasp_pose, pre_grasp_matrix)

        pose_list.append(final_pose)
        return pose_list
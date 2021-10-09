from openravepy import *
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import math
import random
import Config

class PostPutDownPoseGeneratorKeva(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'obj']

        super(PostPutDownPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('obj')
        self.putdown_pose = known_argument_values.get('putdown_pose')
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_pose_list(self):
        # putdown_pose_quat = quatFromRotationMatrix(self.putdown_pose[:3,:3])
        # r = R.from_quat([putdown_pose_quat[1], putdown_pose_quat[2], putdown_pose_quat[3], putdown_pose_quat[0]])
        # euler = r.as_euler('xyz', degrees=True)
        pre_putdown_offset = 0.03
        pose_list =[]
        env = self.simulator.env
        robot = env.GetRobot(Config.ROBOT_NAME)
        pre_putdown_matrix = np.identity(4)
        #if euler[0]<135 and euler[0]>45 and euler[1]<45 and euler[1]>-45 and euler[2]<135 and euler[2]>45:
        pre_putdown_matrix[2][3] -= pre_putdown_offset


        final_pose = np.matmul(self.putdown_pose, pre_putdown_matrix)

        pose_list.append(final_pose)
        return pose_list
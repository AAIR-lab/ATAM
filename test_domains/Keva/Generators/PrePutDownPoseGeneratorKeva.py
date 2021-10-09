from openravepy import *
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import math
import random
import Config

class PrePutDownPoseGeneratorKeva(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'obj']

        super(PrePutDownPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
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
        pre_putdown_offset = 0.03
        env = self.simulator.env
        # pre putdown pose is always offset in z axis in world frame
        pre_putdown_pose = self.putdown_pose.copy()
        pre_putdown_pose[2][3] += pre_putdown_offset

        return [pre_putdown_pose]
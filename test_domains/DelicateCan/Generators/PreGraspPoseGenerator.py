import copy
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import time

class PreGraspPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ["gpose"]

        super(PreGraspPoseGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.gpose = known_argument_values["gpose"]
        self.offset = 0.10 # user defined pregrasp offset

        self._compute_pose_list()
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_pose_list(self):
        env = self.simulator.env
        t0 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, 0, 0, 0)
        t1 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, -self.offset, 0, 0)
        tpgp_wrt_gp = t0.dot(t1)
        pgp = self.gpose.dot(t1)
        if OpenraveUtils.has_ik_to(self.simulator.env, self.simulator.env.GetRobots()[0], pgp):
            return [pgp]
        else:
            return []

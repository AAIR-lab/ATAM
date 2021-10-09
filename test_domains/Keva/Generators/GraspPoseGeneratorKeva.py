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

class GraspPoseGeneratorKeva(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'obj', 'region']

        super(GraspPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('obj')
        self.grasp_region = known_argument_values.get('region')
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()


    def _compute_pose_list(self):

        # ToDo: Transforms from robot base

        pose_list =[]
        grasp_num = 10
        region_length = 0.117475/3.0
        gripper_offset = 0.135 # 0.35 Initial value
        rot_angle = math.pi / 2.0
        #_, region = self.grasp_region.split('_')
        #offset_mean = (region_length * int(region)) - (2*region_length)

        #grasp_offsets = []
        #for i in range(grasp_num):
        #    grasp_offsets.append(random.uniform(offset_mean-region_length/2.0, offset_mean+region_length/2.0))

        self.env = self.simulator.env
        self.robot = self.env.GetRobots()[0]

        self.table = self.env.GetKinBody('table60')
        base_link = self.robot.GetLinks()[0]
        base_transform = base_link.GetTransform()
        object = self.env.GetKinBody(self.object_name)
        object_transform = object.GetTransform()

        # Change the YuMi URDF. World -> BaseLink
        robot_world_transform = self.robot.GetLink('world').GetTransform()

        roll_angle = -math.pi/2
        roll_rot_matrix = np.identity(4)
        roll_rot_matrix[1][1] = math.cos(roll_angle)
        roll_rot_matrix[1][2] = -math.sin(roll_angle)
        roll_rot_matrix[2][1] = math.sin(roll_angle)
        roll_rot_matrix[2][2] = math.cos(roll_angle)

        yaw_angle = -math.pi/2
        yaw_rot_matrix = np.identity(4)
        yaw_rot_matrix[0][0] = math.cos(yaw_angle)
        yaw_rot_matrix[0][1] = -math.sin(yaw_angle)
        yaw_rot_matrix[1][0] = math.sin(yaw_angle)
        yaw_rot_matrix[1][1] = math.cos(yaw_angle)

        gripper_offset_mat = np.identity(4)
        gripper_offset_mat[2][3] = -gripper_offset
        #gripper_offset_object_transform = np.matmul(object_transform, gripper_offset_mat)

        ##rotated_pose = np.matmul(np.matmul(object_transform, roll_rot_matrix), yaw_rot_matrix)

        #final_pose = np.matmul(np.matmul(np.linalg.inv(base_transform), rotated_pose), gripper_offset_mat)
        ##final_pose_ik = np.matmul(np.matmul(np.linalg.inv(robot_world_transform), rotated_pose), gripper_offset_mat)

        object_wrt_robot_world = np.matmul(np.linalg.inv(robot_world_transform), np.matmul(np.matmul(object_transform, np.matmul(roll_rot_matrix, yaw_rot_matrix)), gripper_offset_mat))
        ik_sols = self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(object_wrt_robot_world, check_collisions=True)
        if len(ik_sols)>0:
            pose_list.append(object_wrt_robot_world)

        return pose_list
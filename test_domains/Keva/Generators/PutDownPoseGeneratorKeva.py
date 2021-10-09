import random
from src.DataStructures.Generator import Generator
import src.OpenraveUtils as OpenraveUtils
import numpy as np
from openravepy import *
import math
import copy
import Config
from trac_ik_python.trac_ik import IK



class PutDownPoseGeneratorKeva(Generator):
    root_plank = ''

    def __init__(self, ll_state=None, known_argument_values=None):

        required_values = ['robot', 'obj']

        super(PutDownPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.generate_function_state = self.generate_function()
        self.table_name = 'table60'
        self.object_name = known_argument_values.get('obj')
        #self.putdown_side = known_argument_values.get('region')
        self.reference_structure_path = Config.REFERENCE_STRUCTURE_PATH
        self.robot = self.simulator.env.GetRobots()[0]


    def reset(self):
        if self.object_name == PutDownPoseGeneratorKeva.root_plank:
            PutDownPoseGeneratorKeva.root_plank = ""
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_put_down_pose_list():
            yield gt

    def get_next(self,flag):
        return self.generate_function_state.next()

    def _compute_put_down_pose_list(self):

        if self.object_name == PutDownPoseGeneratorKeva.root_plank:
            PutDownPoseGeneratorKeva.root_plank = ""
        number_poses = 50
        plank_length = 0.117475
        min_dist = plank_length / 3.0
        max_dist = plank_length

        env = self.simulator.env
        table = env.GetKinBody(self.table_name)
        table_transform = table.GetTransform()
        table_surface_transform = copy.deepcopy(table_transform)
        table_surface_transform[2][3] += 0.1
        robot = env.GetRobots()[0]

        table_geom = env.GetKinBody(self.table_name).GetLink('base').GetGeometries()[0]
        table_x, table_y, table_z = table_geom.GetBoxExtents().tolist()

        robot_world_transform = robot.GetLink('world').GetTransform()

        gripper_base_link = robot.GetActiveManipulator().GetEndEffector()
        gripper_base_transform = gripper_base_link.GetTransform()

        grabbed_plank_pose = robot.GetGrabbed()[0].GetTransform()

        plank_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_base_transform), grabbed_plank_pose)

        plank_contat_translation = np.identity(4)
        plank_contat_translation[0][3] = -plank_length / 2.0
        plank_contact_pose_wrt_origin = np.matmul(grabbed_plank_pose, plank_contat_translation)
        # a = misc.DrawAxes(env, gripper_base_transform)
        # ax = misc.DrawAxes(env, grabbed_plank_pose)
        # ax1 = misc.DrawAxes(env, plank_contact_pose_wrt_origin)
        plank_contact_pose_wrt_plank_base = np.linalg.inv(
            np.matmul(np.linalg.inv(plank_contact_pose_wrt_origin), grabbed_plank_pose))
        gripper_pose_wrt_plank_contact = np.linalg.inv(
            np.matmul(plank_pose_wrt_gripper, plank_contact_pose_wrt_plank_base))

        x_limit = table_x - 0.1
        y_limit = table_y - 0.2

        putdown_x = []
        putdown_y = []

        reference_env = Environment()
        reference_env.Load(self.reference_structure_path)

        translation_matrix = np.identity(4)

        pitch_angle = -math.pi / 2.0
        roll_angle = math.pi / 2.0

        pitch_rotation_matrix = np.identity(4)
        roll_rotation_matrix = np.identity(4)

        pitch_rotation_matrix[0][0] = math.cos(pitch_angle)
        pitch_rotation_matrix[0][2] = math.sin(pitch_angle)
        pitch_rotation_matrix[2][0] = -math.sin(pitch_angle)
        pitch_rotation_matrix[2][2] = math.cos(pitch_angle)

        roll_rotation_matrix[1][1] = math.cos(roll_angle)
        roll_rotation_matrix[1][2] = -math.sin(roll_angle)
        roll_rotation_matrix[2][1] = math.sin(roll_angle)
        roll_rotation_matrix[2][2] = math.cos(roll_angle)

        rotation_matrix = np.matmul(pitch_rotation_matrix, roll_rotation_matrix)
        #draw = misc.DrawAxes(env, grabbed_plank_pose)

        put_down_pose_list = []
        if len(PutDownPoseGeneratorKeva.root_plank) == 0:
            for i in range(number_poses):
                putdown_x.append(random.uniform(0.4, 0.6))
                if putdown_x[-1] > 0.5:
                    putdown_y.append(random.uniform(0.0, 0.1))
                else:
                    putdown_y.append(random.uniform(0.0, 0.40))
            # import IPython
            # IPython.embed()
            for x, y in zip(putdown_x, putdown_y):
                print x, y

                put_down_pose = reference_env.GetKinBody(self.object_name).GetTransform()
                put_down_pose[0][3] = x
                put_down_pose[1][3] = y

                put_down_pose_wrt_robot_world = np.matmul(np.linalg.inv(robot_world_transform), put_down_pose)

                putdown_gripper_pose_wrt_origin = np.matmul(put_down_pose_wrt_robot_world, np.linalg.inv(plank_pose_wrt_gripper))
                draw = misc.DrawAxes(env, putdown_gripper_pose_wrt_origin)
                ik_sols = self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(putdown_gripper_pose_wrt_origin, check_collisions=True)
                if len(ik_sols)>0:
                    put_down_pose_list.append(putdown_gripper_pose_wrt_origin)

                # roll_angle = math.pi
                # roll_matrix = np.identity(4)
                # roll_matrix[1][1] = math.cos(roll_angle)
                # roll_matrix[1][2] = -math.sin(roll_angle)
                # roll_matrix[2][1] = math.sin(roll_angle)
                # roll_matrix[2][2] = math.cos(roll_angle)
                #
                # put_down_pose_rotated = np.matmul(put_down_pose, roll_matrix)
                #
                # #dr = misc.DrawAxes(env, put_down_pose_rotated)
                # put_down_pose_wrt_robot_world = np.matmul(np.linalg.inv(robot_world_transform), put_down_pose_rotated)
                #
                # putdown_gripper_pose_wrt_origin = np.matmul(put_down_pose_wrt_robot_world, plank_pose_wrt_gripper)
                #
                # pose = poseFromMatrix(putdown_gripper_pose_wrt_origin)
                # quat = pose[:4]
                # trans = pose[4:7]
                #
                # # draw = misc.DrawAxes(self.env, rotated_pose)
                # urdf_str = self.get_urdf_string()
                # ik_solver = self.set_IK_solver('world', 'gripper_l_base', urdf_str)
                # seed_state = [10.0] * ik_solver.number_of_joints
                # solution = self.get_IK_solution(ik_solver, seed_state, trans, quat)
                # if solution is not None:
                #     put_down_pose_list.append(putdown_gripper_pose_wrt_origin)

            PutDownPoseGeneratorKeva.root_plank = self.object_name

        else:
            reference_root_plank_transform = reference_env.GetKinBody(PutDownPoseGeneratorKeva.root_plank).GetTransform()
            reference_current_plank_transform = reference_env.GetKinBody(self.object_name).GetTransform()
            reference_current_plank_wrt_root_plank = np.matmul(np.linalg.inv(reference_root_plank_transform),
                                                               reference_current_plank_transform)
            root_plank_transform = env.GetKinBody(PutDownPoseGeneratorKeva.root_plank).GetTransform()

            put_down_pose = np.matmul(root_plank_transform, reference_current_plank_wrt_root_plank)

            put_down_pose_wrt_robot_world = np.matmul(np.linalg.inv(robot_world_transform), put_down_pose)
            putdown_gripper_pose_wrt_robot_world = np.matmul(put_down_pose_wrt_robot_world,
                                                             np.linalg.inv(plank_pose_wrt_gripper))

            #draw = misc.DrawAxes(env, putdown_gripper_pose_wrt_robot_world)
            ik_sols = self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(putdown_gripper_pose_wrt_robot_world, check_collisions=True)
            if len(ik_sols)>0:
                put_down_pose_list.append(putdown_gripper_pose_wrt_robot_world)

            # roll_angle = math.pi
            # roll_matrix = np.identity(4)
            # roll_matrix[1][1] = math.cos(roll_angle)
            # roll_matrix[1][2] = -math.sin(roll_angle)
            # roll_matrix[2][1] = math.sin(roll_angle)
            # roll_matrix[2][2] = math.cos(roll_angle)
            #
            # put_down_pose_rotated = np.matmul(put_down_pose, roll_matrix)
            # put_down_pose_wrt_robot_world = np.matmul(np.linalg.inv(robot_world_transform), put_down_pose_rotated)
            # putdown_gripper_pose_wrt_robot_world = np.matmul(put_down_pose_wrt_robot_world,
            #                                                  np.linalg.inv(plank_pose_wrt_gripper))
            #
            # pose = poseFromMatrix(putdown_gripper_pose_wrt_robot_world)
            # quat = pose[:4]
            # trans = pose[4:7]
            #
            # urdf_str = self.get_urdf_string()
            # ik_solver = self.set_IK_solver('world', 'gripper_l_base', urdf_str)
            # seed_state = [0.0] * ik_solver.number_of_joints
            # solution = self.get_IK_solution(ik_solver, seed_state, trans, quat)
            # if solution is not None:
            #     with env:
            #         robot.SetActiveDOFValues(solution)
            #         report = self.get_colliding_objects(env, self.object_name)
            #     if len(report) == 0:
            #         put_down_pose_list.append(putdown_gripper_pose_wrt_robot_world)

        if len(put_down_pose_list) == 0:
            print "No valid solution found"
        random.shuffle(put_down_pose_list)
        random.shuffle(put_down_pose_list)
        return put_down_pose_list
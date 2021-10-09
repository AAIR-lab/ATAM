import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class BaseTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(BaseTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return BaseTrajectory(self.argumnet_name)

    def apply(self,simulator,value,other_generated_values):
        if value is True:
            pass
        else:
            robot = simulator.env.GetRobot(Config.ROBOT_NAME)
            old_active_dofs = robot.GetActiveDOFIndices()
            robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)
            traj = RaveCreateTrajectory(simulator.env, '')
            Trajectory.deserialize(traj, value)
            numWayPoints = traj.GetNumWaypoints()
            lastWayPoint = traj.GetWaypoint(numWayPoints - 1)
            lastWayPointDOFs = simulator.get_joint_values_from_waypoint(lastWayPoint, robot)
            robot.SetActiveDOFValues(lastWayPointDOFs)
            robot.SetActiveDOFs(old_active_dofs)
    
    def execute(self,simulator,value,other_generated_values):
        if value is True:
            pass
        else:
            robot = simulator.env.GetRobot(Config.ROBOT_NAME)
            traj = RaveCreateTrajectory(simulator.env, '')
            Trajectory.deserialize(traj, value)
            with robot:
                robot.GetController().SetPath(traj)
            robot.WaitForController(0)
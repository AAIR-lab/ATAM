import sys,os
################################# System paths ##################################
PROJ_DIR = os.path.abspath(os.path.dirname(__file__))+'/'
HOME_DIR = os.path.expanduser("~") + '/'
TEST_DIR = PROJ_DIR + "test_domains/"
MISC_DIR = PROJ_DIR + "misc/"
PLANNER_DIR = PROJ_DIR + "planners/"

################################# Domain Selection ##################################
# DOMAIN = 'DelicateCan'
# DOMAIN = 'FetchCanPhysicalWorld'
# DOMAIN = 'Hanoi'
# DOMAIN = 'Manufacture'
# DOMAIN = 'KevaLooped'
# DOMAIN = 'TorsenLSD'
# Domain = 'Hangar'
DOMAIN = "Factory"

### FOR Canworld


# NUM_CANS = "2"
# DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/robotics_fetch_canworld_domain.pddl'
# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/new_canworld_'+NUM_CANS+'_cans_problem.pddl'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/new_canworld_'+NUM_CANS+'_cans_problem.output'
# ROBOT_NAME = 'fetch'
# DOMAIN_NAME = "can_world"
# LL_ACTION_CONFIG = PROJ_DIR + 'ActionConfig_canworldV3.json'
# OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/can_world_'+NUM_CANS+'_cans.dae'


### FOR Canworld MDP

# NUM_CANS = "25"
# DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/new_canworld_mdp_'+NUM_CANS+'_cans_5_domain.pddl'
# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/new_canworld_'+NUM_CANS+'_cans_mdp_problem.pddl'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/new_canworld_'+NUM_CANS+'_cans_problem.output'
# OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/can_world_'+NUM_CANS+'_cans.dae'
# ROBOT_NAME = 'fetch'
# DOMAIN_NAME = "can_world"
# LL_ACTION_CONFIG = PROJ_DIR + 'ActionConfig_canworldV3.json'


### FOR Canworld2

# NUM_CANS = '3'
# DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/canworld2_mdp_domain.pddl'
# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/new_canworld2_3_cans_mdp_problem.pddl'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/mdp_robotics_fetch_'+NUM_CANS+'_cans_problem.output'
#
# OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/can_world_'+NUM_CANS+'_cans.dae'
# ROBOT_NAME = 'fetch'
# DOMAIN_NAME = "can_world2"
# LL_ACTION_CONFIG = PROJ_DIR + 'ActionConfigV2_canworld2.json'

# This is not actually a domain but a placeholder name used for testing. should not be uncommented.
# DOMAIN = 'Testing'

################################# General Arguments ##################################

DEBUG = False
SHOW_VIEWER = True  # False if do not want to open the viewer.
RUN_TRAJ = False # To enable final reuslts simulations
STORE_REFINED = True  # true if final refined policy needs to be pickled.
PLOT = False # To plot the refinement profile.

# PRGraph refinement strategies options: "cost_batch_dfs" / "dfs"
PR_STRATEGY = "cost_batch_dfs"
ANYTIME = False
NACTIONS = 2

VIEWER = 'qtcoin'  # name of the viewer.

MAX_TIME = 1000000000
K = 4
HORIZON = 5
# Probability for stochastic choice between updating the model or backtracking.
BACKTRACK_PROB = 1.0

# Planners
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"

# Selection of high-level planner ( FF_PLANNER for PDDL/ LAO_SOLVER for PPDDL)
HL_PLANNER = FF_PLANNER
# HL_PLANNER = LAO_SOLVER

# Options : "WARNING"/"DEBUG"
LOG_LEVEL = 'DEBUG'

OPENRAVE_LL_ENV = "OpenRaveLowLevelState"

PDDL_STATE = 'pddl_state'
HL_STATE_TYPE = "PDDLState"
# LL_STATE_TYPE = OPENRAVE_LL_ENV
LL_STATE_TYPE = "PDDLLowLevelState"

COLLISION_CHECKER = 'pqp'

# IK solvers.
IK_SOLVER = "ik_fast"
# IK_SOLVER = "trac_ik"

# maximum number of iks to check for motion planning
MAX_IKs_TO_CHECK_FOR_MP = 20

# Selection of which motion planner to use for motion planning
# if generating motion plan through OpenRaveSimulator.
OPENRAVE_NATIVE_MOTION_PLANNER = 'OpenRave_NativeMotion_Planner'

# MOTION_PLANNER = OPENRAVE_NATIVE_MOTION_PLANNER
MOTION_PLANNER = "OMPL_RRTConnect"

LL_PLANNER = MOTION_PLANNER
################################# Domain Specific Configs ##################################

# These can override all General Arguments
sys.path.append(TEST_DIR+DOMAIN+'/')
# Config file specific to Domain. Importing here to override general args.

from DomainConfig import *
# Policy path, needed if using PPDDL
POLICY_OUTPUT_FILE = DOMAIN_DIR + "Tasks/graph1.gv"
# Temp file path, needed if using PPDDL
COMBINED_FILE = DOMAIN_DIR + "Tasks/combined_file0.pddl"
# name of the file to store final results. used if PLOT is true.
if PLOT:
    RESULTS_FILE = "results_"+DOMAIN+".csv"

################################# Robot Specific Configs ##################################

YUMI_URDF = MISC_DIR + 'RobotModels/yumi_urdf/yumi_description/urdf/yumi.urdf'
YUMI_SRDF = MISC_DIR + 'RobotModels/yumi_urdf/yumi_description/urdf/yumi.srdf'

FETCH_URDF = MISC_DIR + 'RobotModels/fetch/URDF/fetch.urdf'
FETCH_SRDF = MISC_DIR + 'RobotModels/fetch/URDF/fetch.srdf'
# ROBOT_NAME is retried from DomainConfig import

if ROBOT_NAME == 'fetch':
    ROBOT_URDF = FETCH_URDF
    ROBOT_SRDF = FETCH_SRDF
    ROBOT_MANIP_DOF = 8
elif ROBOT_NAME == 'yumi':
    ROBOT_URDF = YUMI_URDF
    ROBOT_SRDF = YUMI_SRDF
    ROBOT_MANIP_DOF = 7
elif ROBOT_NAME == 'UAV':
    ROBOT_BASE_JOINTS = ["dummy_x","dummy_y","dummy_z"]

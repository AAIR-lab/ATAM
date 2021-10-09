from src.DataStructures.Generator import Generator
import Config

class InitManipPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(InitManipPoseGenerator, self).__init__(known_argument_values=known_argument_values, required_values=None)
        self.generate_function_state = self.generate_function()
        self.simulator = ll_state.simulator
        self.type = Generator.TYPE_QUERY_GENERATOR

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        while True:
            # import IPython
            # IPython.embed()
            robot = self.simulator.env.GetRobot(Config.ROBOT_NAME)
            dof_values = [ 0.00000000e+00, -2.26875349e+00, 2.35601996e+00, 5.23773309e-01,  0.00000000e+00, 6.99004365e-01, -1.74532925e-04]
            yield dof_values

    def get_next(self,flag):
        return self.generate_function_state.next()


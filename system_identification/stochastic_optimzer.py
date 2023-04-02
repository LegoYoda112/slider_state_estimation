import numpy as np

from matplotlib import pyplot as plt


class StochasticOptimizer():
    def __init__(self):

        # Parameters
        self.parameter_num = None
        self.parameter_initial_state = None
        self.parameter_min = None
        self.parameter_max = None
        self.parameter_range = None

        self.current_parameters = None

        self.calc_reward = None

        # How fast to converge to a solution
        self.scale_factor = 1.0
        self.nudge_factor = 0.01
        # TODO: do this better than that
        self.previous_cost = 1000000

    def set_initial_parameters(self, initial_parameters):
        self.parameter_num = len(initial_parameters)
        self.parameter_initial_state = initial_parameters
        self.current_parameters = self.parameter_initial_state

    def set_parameter_range(self, parameter_max, parameter_min):
        self.parameter_min = parameter_min
        self.parameter_max = parameter_max

        self.parameter_range = self.parameter_max - self.parameter_min

    def generate_random_parameter_nudge(self):
        random_nudge = np.random.randn(self.parameter_num)
        # Scale random nudge based on the parameter range
        random_nudge = random_nudge * self.parameter_range * self.nudge_factor

        return random_nudge

    def set_calc_cost(self, cost_function):
        self.calc_cost = cost_function

    def perform_step(self):
        nudge = self.generate_random_parameter_nudge()

        trial_parameters = self.current_parameters + nudge

        # Clamp to min max
        trial_parameters = np.minimum(trial_parameters, self.parameter_max)
        trial_parameters = np.maximum(trial_parameters, self.parameter_min)
        
        cost = self.calc_cost(trial_parameters)

        # If the cost is better, move a bit in that direction
        if(cost < self.previous_cost):
            self.current_parameters = trial_parameters
            # self.current_parameters += nudge * self.scale_factor
            # self.current_parameters = np.minimum(self.current_parameters, self.parameter_max)
            # self.current_parameters = np.maximum(self.current_parameters, self.parameter_min)
            self.previous_cost = cost

# opt = StochasticOptimizer()

# opt.set_initial_parameters(np.array([1.0, -3.0]))
# opt.set_parameter_range(np.array([-15., -5.]),
#                        -np.array([-3., 3.]))

# def cost_function(param):
#     x1 = param[0]
#     x2 = param[1]
#     return 2*x1**2 - 1.05*x1**4 + 1/6 * x1**6 + x1*x2 + x2**2

# def bukin_function(param):
#     x1 = param[0]
#     x2 = param[1]
#     return 100 * np.sqrt(np.abs(x2 - 0.01 * x1**2)) + 0.01 * np.abs(x1 + 10)

# opt.set_calc_cost(cost_function)

# cost = []
# params = []

# for i in range(2000):
#     #print("Params", opt.current_parameters)
#     #print("Cost", opt.calc_cost(opt.current_parameters))
#     #print()

#     cost.append(opt.calc_cost(opt.current_parameters))
#     params.append(opt.current_parameters.copy())

#     opt.perform_step()

# params = np.array(params)

# plt.subplot(211)
# plt.plot(cost, label = "cost")
# plt.legend()

# plt.subplot(212)
# plt.plot(params)
# # plt.plot(params[:, 1])
# plt.show()
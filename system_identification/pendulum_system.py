import numpy as np
import os

from matplotlib import pyplot as plt
import matplotlib.animation as animation

import pydot

from IPython.display import SVG, display

from pydrake.common import temp_directory
from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    StartMeshcat,
)

from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import ModelVisualizer
from pydrake.systems.primitives import LogVectorOutput
from pydrake.systems.controllers import PidController
from pydrake.multibody.tree import RigidBody

import glob
import imageio.v2

test_mode = True if "TEST_SRCDIR" in os.environ else False

# Define a simple cylinder model.
pendulum_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="pendulum">
    <pose>0 0 0 0 0 0</pose>
    <link name = "pendulum_link">
      <inertial>
        <mass>10.0</mass>
      </inertial>
      <visual name="base">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
      </visual>
      <collision name="base_col">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="world_to_upper_pin_joint" type="revolute">
      <parent>world</parent>
      <child>pendulum_link</child>
      <pose>0 0 0.5 0 0 0</pose>
      <axis>
        <!-- spins about x axis -->
        <xyz>1 0 0</xyz>
      </axis>
    </joint>


  </model>
</sdf>
"""
def create_scene(params, sim_time_step = 0.0001):

    # meshcat.Delete()
    # meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step = sim_time_step
    )
    parser = Parser(plant)

    parser.AddModelsFromString(pendulum_sdf, "sdf")
    # parser.AddModelsFromString(table_top_sdf, "sdf")

    # table_frame = plant.GetFrameByName("table_top_center")
    # plant.WeldFrames(plant.world_frame(), table_frame)

    plant.Finalize()
    plant_context = plant.CreateDefaultContext()

    # Add a PID controller.
    controller = builder.AddNamedSystem("controller",
                                    PidController(kp=[params[0]], ki=[0], kd=[params[1]]))
    
    builder.Connect(plant.get_state_output_port(),
                controller.get_input_port_estimated_state())
    builder.Connect(controller.get_output_port_control(), plant.get_actuation_input_port())

    builder.ExportInput(controller.get_input_port_desired_state())

    # plant.get_actuation_input_port().FixValue(plant_context, np.zeros(plant.num_actuators()))

    # cylinder = plant.GetBodyByName("cylinder_link")
    # X_WorldTable = table_frame.CalcPoseInWorld(plant_context)
    # X_TableCylinder = RigidTransform(
    #     RollPitchYaw(np.asarray([90, 0, 0]) * np.pi / 180), p = [0,0,1.0]
    # )
    # X_WorldCylinder = X_WorldTable.multiply(X_TableCylinder)
    # plant.SetDefaultFreeBodyPose(cylinder, X_WorldCylinder)

    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    logger.set_name("logger")

    # visualizer = MeshcatVisualizer.AddToBuilder(
    #     builder, scene_graph, meshcat,
    #     MeshcatVisualizerParams(role = Role.kPerception, prefix="visual")
    # )

    visualizer = None

    diagram = builder.Build()

    return diagram, visualizer, plant, logger

def initialize_simulation(diagram, plant, params):
    simulator = Simulator(diagram)

    context = simulator.get_mutable_context()
    default_state_vector = context.get_mutable_discrete_state_vector()
    # print(default_state_vector)

    cylinder_body = plant.GetRigidBodyByName("pendulum_link")
    RigidBody.SetMass(cylinder_body, context, 2.0)

    # print(context)
    context.SetDiscreteState(np.array([0.9, 0]))

    diagram.get_input_port(0).FixValue(context, [0.0, 0.])

    # Fix input port to zero
    # plant_context = diagram.GetMutableSubsystemContext(
    #     plant, simulator.get_mutable_context()
    # )
    # plant.get_actuation_input_port().FixValue(
    #     plant_context, np.zeros(plant.num_actuators())
    # )

    #display(SVG(pydot.graph_from_dot_data(
    #    diagram.GetGraphvizString(max_depth=2))[0].create_svg()))

    simulator.Initialize()
    # simulator.set_target_realtime_rate(1,)
    return simulator

def run_simulation(sim_time_step, end_time, params):
    diagram, visualizer, plant, logger = create_scene(params, sim_time_step)
    simulator = initialize_simulation(diagram, plant, params)
    # visualizer.StartRecording()
    simulator.AdvanceTo(end_time)
    # visualizer.PublishRecording

    log = logger.FindLog(simulator.get_context())
    t = log.sample_times()

    return log.data()

initial_params = [10.0, 1.0]

def generate_initial_trajectory():
    params = [10.0, 1.0]
    return generate_trajectory(params)

def generate_trajectory(params):
    data = run_simulation(sim_time_step=0.01, end_time = 10.0, params = params)
    return data

ref_trajectory = generate_initial_trajectory()

def mse(array_1, array_2):
    array_1 = array_1.flatten()
    array_2 = array_2.flatten()
    difference_array = np.subtract(array_1, array_2)
    squared_array = np.square(difference_array)
    mse = squared_array.mean()

    print(mse)

    return mse

def calc_param_cost(params):

    # print(len(ref_trajectory[0]))
    new_trajectory = generate_trajectory(params)
    # print(len(new_trajectory[0]))

    # print(mse(ref_trajectory, new_trajectory))

    return mse(ref_trajectory, new_trajectory)

# Black box param cost
# calc_param_cost([0.0])

from stochastic_optimzer import StochasticOptimizer

opt = StochasticOptimizer()

opt.set_initial_parameters(np.array([0.0, 0.0]))
opt.set_parameter_range(np.array([20., 20.]),
                        np.array([0.,  0.]))

opt.set_calc_cost(calc_param_cost)


cost = []
params = []

def make_plot(num):
  params_plot = np.array(params)

  plt.figure(figsize=[10,5], dpi=150)
  plt.subplot(223)
  plt.plot(cost, label = "error")
  plt.legend()

  plt.subplot(224)

  plt.hlines(initial_params, 0, len(cost))

  for i in range(len(params_plot[0])):
      plt.plot(params_plot[:, i], label = i)
    
  plt.legend()

  plt.subplot(221)
  plt.title("Original Trajectory")
  plt.plot(ref_trajectory[0,:])
  plt.plot(ref_trajectory[1,:])

  plt.subplot(222)
  plt.title("Trial Trajectory")
  final_trajectory = generate_trajectory(opt.current_parameters)
  plt.plot(final_trajectory[0,:])
  plt.plot(final_trajectory[1,:])
  plt.savefig(f'./plots/plt-{str(num).zfill(3)}.png')

for i in range(300):
    
    cost.append(opt.calc_cost(opt.current_parameters))
    params.append(opt.current_parameters.copy())

    if(not i % 5):
      make_plot(i)

    opt.perform_step()

images = sorted(glob.glob("./plots/*"))

with imageio.get_writer('line.gif', mode='i') as writer:
    for image in images:
        image = imageio.imread(image)
        writer.append_data(image)

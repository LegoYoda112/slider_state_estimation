#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState

import numpy as np
import os

# Pydrake imports
from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    StartMeshcat,
)
from pydrake.common.eigen_geometry import Quaternion as drake_Quaternion
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from ament_index_python.packages import get_package_share_directory
# may raise PackageNotFoundError
package_share_directory = get_package_share_directory('slider_state_estimation')

# CONSTANTS
slider_main_model_file = package_share_directory +  "/models/urdf/slider_2_0_urdf.urdf"
slider_aux_model_file = package_share_directory + "/models/urdf/slider_2_0_urdf-transparent.urdf"

class BodyPoseEstimator(Node):

    def __init__(self):
        super().__init__('body_pose_estimator')

        self.get_logger().info("Starting body pose estimator")

        self.pose_publisher = self.create_publisher(PoseStamped, '/slider/state_estimation/body_pose', 10)
        self.vel_publisher = self.create_publisher(Vector3Stamped, '/slider/state_estimation/body_velocity', 10)
        
        self.motor_state_subscriber = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.motor_state_callback,
            10)
        
        self.body_orientation_subscriber = self.create_subscription(
            Quaternion, 
            '/slider/sensors/imu/orientation', 
            self.body_orientation_callback,
            10)

        self.joint_positions = np.zeros(10)

        self.meshcat = StartMeshcat()

        # Drake setup
        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(self.plant)

        self.slider_main = parser.AddModelFromFile(slider_main_model_file, model_name="slider_main")
        self.slider_aux = parser.AddModelFromFile(slider_aux_model_file, model_name="slider_aux")

        meshcat_vis = MeshcatVisualizer.AddToBuilder(builder = builder, scene_graph = self.scene_graph, meshcat = self.meshcat)
        
        self.plant.Finalize()
        self.diagram = builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()

        self.previous_contact_frame = 'Left_Foot'
        self.contact_point = [0,0,0]

        self.velocity_estimate = [0, 0, 0]
        self.previous_body_pos = [0, 0, 0]
        self.previous_run_time = self.get_clock().now()

        self.setup_drake_joints()

    # Grabs the robot joint handles from drake
    def setup_drake_joints(self):
        self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)

        # Main Robot
        self.base_link_main = self.plant.GetJointByName("$world_base_link", self.slider_main)

        self.left_roll_main = self.plant.GetJointByName('Left_Roll', self.slider_main)
        self.left_pitch_main = self.plant.GetJointByName('Left_Pitch', self.slider_main)
        self.left_slide_main = self.plant.GetJointByName('Left_Slide', self.slider_main)
        self.left_foot_roll_main = self.plant.GetJointByName('Left_Foot_Roll', self.slider_main)
        self.left_foot_pitch_main = self.plant.GetJointByName('Left_Foot_Pitch', self.slider_main)

        self.right_roll_main = self.plant.GetJointByName('Right_Roll', self.slider_main)
        self.right_pitch_main = self.plant.GetJointByName('Right_Pitch', self.slider_main)
        self.right_slide_main = self.plant.GetJointByName('Right_Slide', self.slider_main)
        self.right_foot_roll_main = self.plant.GetJointByName('Right_Foot_Roll', self.slider_main)
        self.right_foot_pitch_main = self.plant.GetJointByName('Right_Foot_Pitch', self.slider_main)


        # Aux Robot
        self.base_link_aux = self.plant.GetJointByName("$world_base_link", self.slider_aux)

        self.left_roll_aux = self.plant.GetJointByName('Left_Roll', self.slider_aux)
        self.left_pitch_aux = self.plant.GetJointByName('Left_Pitch', self.slider_aux)
        self.left_slide_aux = self.plant.GetJointByName('Left_Slide', self.slider_aux)
        self.left_foot_roll_aux = self.plant.GetJointByName('Left_Foot_Roll', self.slider_aux)
        self.left_foot_pitch_aux = self.plant.GetJointByName('Left_Foot_Pitch', self.slider_aux)

        self.right_roll_aux = self.plant.GetJointByName('Right_Roll', self.slider_aux)
        self.right_pitch_aux = self.plant.GetJointByName('Right_Pitch', self.slider_aux)
        self.right_slide_aux = self.plant.GetJointByName('Right_Slide', self.slider_aux)
        self.right_foot_roll_aux = self.plant.GetJointByName('Right_Foot_Roll', self.slider_aux)
        self.right_foot_pitch_aux = self.plant.GetJointByName('Right_Foot_Pitch', self.slider_aux)

    # Sets the main robot's joint angles in drake
    def drake_set_q_main(self, q):
        self.left_roll_main.set_angle(self.plant_context, q[0]),
        self.left_pitch_main.set_angle(self.plant_context, q[1])
        self.left_slide_main.set_translation(self.plant_context, q[2])
        self.left_foot_roll_main.set_angle(self.plant_context, q[3])
        self.left_foot_pitch_main.set_angle(self.plant_context, q[4])

        self.right_roll_main.set_angle(self.plant_context, q[5])
        self.right_pitch_main.set_angle(self.plant_context, q[6])
        self.right_slide_main.set_translation(self.plant_context, q[7])
        self.right_foot_roll_main.set_angle(self.plant_context, q[8])
        self.right_foot_pitch_main.set_angle(self.plant_context, q[9])

    # Gets the main robot's angles in drake
    def get_q_main(self):
        q_main = [
            self.left_roll_main.get_angle(self.plant_context),
            self.left_pitch_main.get_angle(self.plant_context),
            self.left_slide_main.get_translation(self.plant_context),
            self.left_foot_roll_main.get_angle(self.plant_context),
            self.left_foot_pitch_main.get_angle(self.plant_context),

            self.right_roll_main.get_angle(self.plant_context),
            self.right_pitch_main.get_angle(self.plant_context),
            self.right_slide_main.get_translation(self.plant_context),
            self.right_foot_roll_main.get_angle(self.plant_context),
            self.right_foot_pitch_main.get_angle(self.plant_context)
        ]
        return q_main
    
    def get_world_position_of_frame(self,name, robot):
        frame = self.plant.GetFrameByName(name, robot)
        frame_pose = frame.CalcPoseInWorld(self.plant_context)
        frame_position = np.array(frame_pose.translation())

        return frame_position
    
    def calculate_rel_robot_position(self, quaternion, q, initial = False):

        self.base_link_aux.set_quaternion(self.plant_context, quaternion)

        # Copy joint angles
        self.left_roll_aux.set_angle(self.plant_context, q[0])
        self.left_pitch_aux.set_angle(self.plant_context, q[1])
        self.left_slide_aux.set_translation(self.plant_context, q[2])
        self.left_foot_roll_aux.set_angle(self.plant_context, q[3])
        self.left_foot_pitch_aux.set_angle(self.plant_context, q[4])

        self.right_roll_aux.set_angle(self.plant_context, q[5])
        self.right_pitch_aux.set_angle(self.plant_context, q[6])
        self.right_slide_aux.set_translation(self.plant_context, q[7])
        self.right_foot_roll_aux.set_angle(self.plant_context, q[8])
        self.right_foot_pitch_aux.set_angle(self.plant_context, q[9])

        base_frame_position = self.get_world_position_of_frame("base_link", self.slider_aux)

        left_foot_world_position = self.get_world_position_of_frame("Left_Foot", self.slider_aux)
        left_foot_rel_position = left_foot_world_position - base_frame_position
        right_foot_world_position = self.get_world_position_of_frame("Right_Foot", self.slider_aux)
        right_foot_rel_position = right_foot_world_position - base_frame_position

        # If left foot is lower than right foot, assume it is in contact with the ground

        if(left_foot_world_position[2] < right_foot_world_position[2]):
            self.contact_offset = left_foot_rel_position
            self.current_contact_frame = "Left_Foot"
        else:
            self.contact_offset = right_foot_rel_position
            self.current_contact_frame = "Right_Foot"

        if(self.current_contact_frame != self.previous_contact_frame and not initial):
            prev_contact_offset = self.get_world_position_of_frame(self.previous_contact_frame, self.slider_aux) - base_frame_position
            self.base_link_aux.set_position(self.plant_context, self.contact_point - prev_contact_offset)

            self.contact_point = self.get_world_position_of_frame(self.current_contact_frame, self.slider_aux)
        
        if(initial):
            self.contact_point = [0, 0, 0]

        # Set contact point on the ground
        self.contact_point[2] = 0

        self.base_link_aux.set_position(self.plant_context, self.contact_point - self.contact_offset)

        self.previous_contact_frame = self.current_contact_frame

    
    def motor_state_callback(self, msg):
        joint_states = msg

        #print(joint_states.name)

        # Loop through recived data and set position
        for i in range(10):
            self.joint_positions[i] = joint_states.position[i]
        
        left_right_swapped = [*self.joint_positions[5:10], *self.joint_positions[0:5]]
        # print(left_right_swapped)
        
        self.drake_set_q_main(left_right_swapped)

    def body_orientation_callback(self, msg):
        body_orientation = msg
        quat = [body_orientation.w, body_orientation.x, body_orientation.y, body_orientation.z]
        
        # Update robot pose estimate
        quat = quat / np.linalg.norm(quat)
        drake_quat = drake_Quaternion(quat)
        self.base_link_main.set_quaternion(self.plant_context, drake_quat)

        self.calculate_rel_robot_position(self.base_link_main.get_quaternion(self.plant_context), self.get_q_main())
        self.diagram.ForcedPublish(self.diagram_context)

        body_pose_msg = PoseStamped()
        body_pose_msg.header.stamp = self.get_clock().now().to_msg()
        body_pose_msg.header.frame_id = "world"

        body_pose_msg.pose.orientation = body_orientation

        body_position = self.get_world_position_of_frame("base_link", self.slider_aux)
        body_pose_msg.pose.position.x = body_position[0]
        body_pose_msg.pose.position.y = body_position[1]
        body_pose_msg.pose.position.z = body_position[2]

        self.pose_publisher.publish(body_pose_msg)

        ns_to_s = 1e9

        time_dt = (self.get_clock().now() - self.previous_run_time).nanoseconds / ns_to_s
        self.velocity_estimate = (body_position - self.previous_body_pos) / time_dt
        self.previous_body_pos = body_position

        self.previous_run_time = self.get_clock().now()

        body_vel_msg = Vector3Stamped()
        body_vel_msg.header.stamp = self.get_clock().now().to_msg()
        body_vel_msg.header.frame_id = "world"

        body_vel_msg.vector.x = self.velocity_estimate[0]
        body_vel_msg.vector.y = self.velocity_estimate[1]
        body_vel_msg.vector.z = self.velocity_estimate[2]

        self.vel_publisher.publish(body_vel_msg)



def main(args=None):
    rclpy.init(args=args)

    body_pose_estimator = BodyPoseEstimator()

    rclpy.spin(body_pose_estimator)

    body_pose_estimator.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()
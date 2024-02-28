import pybullet as p
import pybullet_data

import numpy as np
import pandas as pd

import math

import os
import sys

path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")
)  # this is a bit hacky... just in case the user doesnt have somo installed...
# sys.path.insert(0, path)

from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.sm_actuator_definition import SMActuatorDefinition
from somo.sm_link_definition import SMLinkDefinition
from somo.sm_joint_definition import SMJointDefinition
from somo.sm_continuum_manipulator import SMContinuumManipulator

from somo.utils import load_constrained_urdf

import sorotraj

# select whether you want to record a video or not
VIDEO_LOGGING = False

######## SIMULATION SETUP ########
### prepare everything for the physics client / rendering
## Pretty rendering
opt_str = "--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0"  # this opens the gui with a white background and no ground grid
opt_str = ""
cam_width, cam_height = 1920, 1640
if cam_width is not None and cam_height is not None:
    opt_str += " --width=%d --height=%d" % (cam_width, cam_height)

physicsClient = p.connect(
    p.GUI, options=opt_str
)  # starts the physics client with the options specified above. replace p.GUI with p.DIRECT to avoid gui

p.setPhysicsEngineParameter(enableFileCaching=0)

# This two lines are unnecessary since the simulation is reset before each run
"""
plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, plane)
"""

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)  # disable rendering while loading objects
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) #Enable GUI widgets
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0) #disable TinyRenderer

# Set the camera position. This goes right after you instantiate the GUI:
# cam_distance, cam_yaw, cam_pitch, cam_xyz_target = 3, -30.0, -30, [0.0, 0.0, 0.0]
cam_distance, cam_yaw, cam_pitch, cam_xyz_target = 3, -30, -89, [0.0, 0.0, 0.0]
p.resetDebugVisualizerCamera(
    cameraDistance=cam_distance,
    cameraYaw=cam_yaw,
    cameraPitch=cam_pitch,
    cameraTargetPosition=cam_xyz_target,
)

## Set physics parameters and simulation properties
p.setGravity(0, 0, -9.8)
p.setPhysicsEngineParameter(enableConeFriction=1)
p.setRealTimeSimulation(
    0
)  # this is necessary to enable torque control. only if this is set to 0 and the simulation is done with explicit steps will the torque control work correctly

## Specify time steps
time_step = 0.001
p.setTimeStep(time_step)
n_steps = 20000

### load all the objects into the environment

# set ground plane friction, unnecessary because of the reset
""" p.changeDynamics(plane, -1, lateralFriction=1)""" 


snake_yaml = os.path.join(os.path.dirname(__file__), "definitions", "bb_snake.yaml")
snake_yaml_2 = os.path.join(os.path.dirname(__file__), "definitions", "snake_discrete.yaml")
arm_manipulator_def = SMManipulatorDefinition.from_file(snake_yaml)

# create the arm manipulator...
arm = SMContinuumManipulator(arm_manipulator_def, testFlag=0)
# ... and load it, most are unnecssary due to the reset
startPos = [0, 0, 0]
startOr = p.getQuaternionFromEuler([-np.pi/2, 0, -np.pi/2])

arm.load_to_pybullet(
    baseStartPos=startPos,
    baseStartOrn=startOr,
    # baseConstraint="static",  # other options are free and constrained, but those are not recommended rn
    baseConstraint="free",
    physicsClient=physicsClient,
)

# below is an example of how lateral friction and restitution can be changed for the whole manipulator.

# get the link index of the manipulator. In somo, the robot_id is called bodyUniqueId
jointsCount = p.getNumJoints(arm.bodyUniqueId)
linksCount = jointsCount
simulationCount = pow(2, linksCount)
for joint_index in range(jointsCount):
    joint_info = p.getJointInfo(arm.bodyUniqueId, joint_index)
    link_name = joint_info[12].decode("utf-8")
    print(f"Link {joint_index}: {link_name}")


contact_properties_with_fri = {
    "lateralFriction": 1,
    "anisotropicFriction": [12, 0.01, 0.01],
    "angularDamping": 3,
    'restitution': 3.0, # uncomment to change restitution
}

# arm.set_contact_property(contact_properties_with_fri)
# Use linkIndex to specify the link you want to change the contact properties of
contact_properties_without_fri = {
    "lateralFriction": 1,
    "anisotropicFriction": [0.01, 0.01, 0.01],
    "angularDamping": 3,
    'restitution': 3.0, # uncomment to change restitution
}

# arm.set_contact_property_for_link(contact_properties_without_fri, linkIndex=1, linkNum=0)

######## PRESCRIBE A TRAJECTORY ########
# here, the trajectory is hard-coded (booh!) and prepared using the sorotraj format
traj = sorotraj.TrajBuilder(graph=False)
trajectory_loop = os.path.join(os.path.dirname(__file__), "trajectory.yaml")
traj.load_traj_def(trajectory_loop)
trajectory = traj.get_trajectory()
interp = sorotraj.Interpolator(trajectory)
actuation_fn = interp.get_interp_function(
    num_reps=20, speed_factor=1, invert_direction=False, as_list=False
)

######## EXECUTE SIMULATION ########
# if desired, start video logging - this goes before the run loop
if VIDEO_LOGGING:
    vid_filename = "~/vid.mp4"
    logIDvideo = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, vid_filename)

# create a .csv file to store the configuration and speed.
# if the file already exists, delete it and create a new one.
if os.path.exists(f"{os.path.dirname(__file__)}/speeds.csv"):
        os.remove(f"{os.path.dirname(__file__)}/speeds.csv")
with open(f"{os.path.dirname(__file__)}/speeds.csv", "w") as f:
    f.write("Configuration,X-linear Velocity,Y-linear Velocity,Speed,Yaw Rate\n")

# execute the simulation multiple times with different friction configurations
simulationCount = 1 # This is for testing.....
for curSimulation in range(simulationCount):
    # curSimulation = int('0000111110000011', 2) # Test some specific configuration
    p.resetSimulation()

    # create the ground plane
    plane = p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, plane)
    p.changeDynamics(plane, -1, lateralFriction=1)

    # set physics parameters
    p.setGravity(0, 0, -9.8)
    p.setPhysicsEngineParameter(enableConeFriction=1)
    p.setRealTimeSimulation(0)
    
    # reset the robot
    arm.load_to_pybullet(
        baseStartPos=startPos,
        baseStartOrn=startOr,
        # baseConstraint="static",  # other options are free and constrained, but those are not recommended rn
        baseConstraint="free",
        physicsClient=physicsClient,
    )

    # process the friction configuration
    curSimulationBinary = bin(curSimulation)[2:]
    curSimulationBinary = curSimulationBinary.zfill(linksCount)
    for link_index in range(linksCount):
        if curSimulationBinary[link_index] == '1':
            arm.set_contact_property_for_link(contact_properties_without_fri, linkIndex=link_index, linkNum=1)
        else:
            arm.set_contact_property_for_link(contact_properties_with_fri, linkIndex=link_index, linkNum=1)

    # enable rendering
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1) 
    
    TrajectoryData = []

    for i in range(n_steps * 100):

        pos, orn = p.getBasePositionAndOrientation(arm.bodyUniqueId)
        TrajectoryData.append([pos, orn])

        torques = actuation_fn(
            i * time_step
        )  # retrieve control torques from the trajectory.
        # print(f"i = {i}\t{torques}")
        # applying the control torques
        
        """
        arm.apply_actuation_torques(
            actuator_nrs=[0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7],
            axis_nrs=[0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
            actuation_torques=torques.tolist(),
        )
        """

        p.stepSimulation()

    # get the many kinds of velocity
    linear, angular = p.getBaseVelocity(arm.bodyUniqueId)
    speed = np.linalg.norm(linear)
    print(f"linear velocity: {linear}\tspeed: {speed}\tangular velocity: {angular}")

    # print the similar friction configurations and its speed
    # comment this when the simulationCount gets large.

    arm.print_similar_friction_configurations(curSimulationBinary)

    # write the configuration binary name with the speed to the .csv file
    # if you want to save the velocity data to a .csv file, uncomment the following lines
    '''
    with open(f"{os.path.dirname(__file__)}/speeds.csv", "a") as f:
        f.write(f"'{curSimulationBinary},{linear[0]},{linear[1]},{speed},{angular[2]}\n")
    '''

    # if you want to save the velocity data to a .csv or .png file, uncomment the following lines
    '''
    arm.delete_position_files(curSimulationBinary)
    arm.save_position_as_csv(TrajectoryData, suffix=str(curSimulationBinary))
    arm.save_position_as_png(TrajectoryData, suffix=str(curSimulationBinary))
    '''
    
    
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)


######## CLEANUP AFTER SIMULATION ########
# this goes after the run loop
if VIDEO_LOGGING:
    p.stopStateLogging(logIDvideo)
# ... aaand disconnect pybullet
p.disconnect()

import pybullet as p
import pybullet_data

import numpy as np
import pandas as pd

import os, time
from multiprocessing import Process

path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")
)  # this is a bit hacky... just in case the user doesnt have somo installed...
# sys.path.insert(0, path)

from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.sm_continuum_manipulator import SMContinuumManipulator

from somo.utils import load_constrained_urdf

import sorotraj

# select whether you want to record a video or not
VIDEO_LOGGING = False

######## SIMULATION SETUP ########
### prepare everything for the physics client / rendering
## Pretty rendering
"""
opt_str = "--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0"  # this opens the gui with a white background and no ground grid
opt_str = ""
cam_width, cam_height = 1920, 1640
if cam_width is not None and cam_height is not None:
    opt_str += " --width=%d --height=%d" % (cam_width, cam_height)
"""

# Set the camera position. This goes right after you instantiate the GUI:
    # cam_distance, cam_yaw, cam_pitch, cam_xyz_target = 3, -30.0, -30, [0.0, 0.0, 0.0]
DebugVisualizerCamera = [3, -30, -89, [0.0, 0.0, 0.0]]

contact_properties_with_fri = {
    "lateralFriction": 1,
    "anisotropicFriction": [12, 0.01, 0.01],
    "angularDamping": 3,
    'restitution': 3.0, # uncomment to change restitution
}

# Load the manipulator definition and trajectory
snake_yaml = os.path.join(os.path.dirname(__file__), "definitions", "bb_snake.yaml")
## snake_yaml_2 = os.path.join(os.path.dirname(__file__), "definitions", "snake_discrete.yaml")
arm_manipulator_def = SMManipulatorDefinition.from_file(snake_yaml)
arm = SMContinuumManipulator(arm_manipulator_def, testFlag=0)


######## PRESCRIBE A TRAJECTORY ########
# here, the trajectory is hard-coded (booh!) and prepared using the sorotraj format
def acquire_actuation(filename: str):
    traj = sorotraj.TrajBuilder(graph=False)
    trajectory_loop = os.path.join(os.path.dirname(__file__), filename)
    traj.load_traj_def(trajectory_loop)
    trajectory = traj.get_trajectory()
    interp = sorotraj.Interpolator(trajectory)
    actuation_fn = interp.get_interp_function(
        num_reps=20, speed_factor=1, invert_direction=False, as_list=False
    )
    return actuation_fn

actuation_function = acquire_actuation("trajectory.yaml")

def runSimulation(
        arm : SMContinuumManipulator,
        frictionConfigurations,
        actuation_fn,
        DebugVisualizerCamera = None,
        ConnectMethods = "DIRECT",
        opt_str = ""
):
    if ConnectMethods == "GUI":
        physicsClient = p.connect(
            p.GUI, options=opt_str
        )
    elif ConnectMethods == "DIRECT":
        physicsClient = p.connect(
            p.DIRECT
        )
    else:
        raise ValueError("ConnectMethods must be either 'GUI' or 'DIRECT'")

    if DebugVisualizerCamera:
        p.resetDebugVisualizerCamera(
            cameraDistance=DebugVisualizerCamera[0],
            cameraYaw=DebugVisualizerCamera[1],
            cameraPitch=DebugVisualizerCamera[2],
            cameraTargetPosition=DebugVisualizerCamera[3],
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

    # load it
    startPos = [0, 0, 0]
    startOr = p.getQuaternionFromEuler([-np.pi/2, 0, -np.pi/2])

    arm.load_to_pybullet(
        baseStartPos=startPos,
        baseStartOrn=startOr,
        # baseConstraint="static",  # other options are free and constrained, but those are not recommended rn
        baseConstraint="free",
        physicsClient=physicsClient,
    )


    # get the link index of the manipulator. In somo, the robot_id is called bodyUniqueId
    jointsCount = p.getNumJoints(arm.bodyUniqueId)
    linksCount = jointsCount
    for joint_index in range(jointsCount):
        joint_info = p.getJointInfo(arm.bodyUniqueId, joint_index)
        link_name = joint_info[12].decode("utf-8")

    for link in range(linksCount):
        arm.set_contact_property_for_link(frictionConfigurations, linkIndex=link, linkNum=1)

    

    # create the ground plane
    plane = p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, plane)
    p.changeDynamics(plane, -1, lateralFriction=1)

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
        
        arm.apply_actuation_torques(
            actuator_nrs=[0, 0, 1, 1],
            axis_nrs=[0, 1, 0, 1],
            actuation_torques=torques.tolist(),
        )

        p.stepSimulation()

    # get the many kinds of velocity
    linear, angular = p.getBaseVelocity(arm.bodyUniqueId)
    speed = np.linalg.norm(linear)
    print(f"linear velocity: {linear}\tspeed: {speed}\tangular velocity: {angular}")
    
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    ######## CLEANUP AFTER SIMULATION ########
    p.disconnect()

start = time.time()
for i in range(50):
    runSimulation(arm = arm,
                frictionConfigurations = contact_properties_with_fri,
                trajectory_loop = actuation_function,
                )
end = time.time()
print(f"single processing runtime: {end - start}")

start = time.time()
processArray = {}
for pid in range(50):
    processArray[pid] = Process(target=runSimulation, args=(arm, contact_properties_with_fri, actuation_function))
    processArray[pid].start()

for pid in range(50):
    processArray[pid].join()
end = time.time()
print(f"multi processing runtime: {end - start}")

# Generate ramdom friction configurations in binary code, in the range of 0 to 90 for each link



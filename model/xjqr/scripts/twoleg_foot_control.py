import numpy as np
import pybullet as p
import pybullet_data

from isaacsim import SimulationApp

import carb
import sys


# Initialize Isaac Sim
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics, Gf, UsdGeom, UsdLux
import omni.usd

class LegTrajectoryPlanner:
    def __init__(self, robot_id, joint_indices, end_effector_link_index, leg_name="Leg"):
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.endEffectorLinkIndex = end_effector_link_index
        self.leg_name = leg_name
        self.lowerLimits = [-2.093, -2.093, -3.14, -2.093, -3.14, -3.14,
                            -2.093, -2.093, -3.14, -2.093, -3.14, -3.14]
        self.upperLimits = [2.093, 2.093, 3.14, 2.093, 3.14, 3.14,
                            2.093, 2.093, 3.14, 2.093, 3.14, 3.14]

    def getCurrentJointValues(self):
        return [p.getJointState(self.robot_id, idx)[0] for idx in self.joint_indices]

    def calcIk(self, target_pos, target_quat):
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            self.endEffectorLinkIndex,
            targetPosition=target_pos,
            targetOrientation=target_quat
        )
        return joint_poses

    def getTrajectory(self, start_joint_value, target_pos, target_quat, t, point_num):
        full_joint_solution = self.calcIk(target_pos, target_quat)
        q0 = np.array(start_joint_value)
        qf = np.array([full_joint_solution[i] for i in self.joint_indices])

        t0 = 0
        tf = t
        n = point_num
        t_list = np.linspace(t0, tf, n)

        T = np.array([[1, t0, t0**2, t0**3],
                      [0, 1, 2*t0, 3*t0**2],
                      [1, tf, tf**2, tf**3],
                      [0, 1, 2*tf, 3*tf**2]])

        Q = np.zeros((4, 6))
        Q[0, :] = q0
        Q[2, :] = qf

        A = np.zeros((4, 6))
        for i in range(6):
            A[:, i] = np.linalg.solve(T, Q[:, i])

        Qd = np.zeros((n, 6))
        for i in range(n):
            Td = np.array([1, t_list[i], t_list[i]**2, t_list[i]**3])
            Qd[i, :] = Td.dot(A)

        return Qd, t_list

def leg_joint_control(leg_prim_path, target_positions):
    stage = omni.usd.get_context().get_stage()
    
    for joint_path, position in zip(leg_prim_path, target_positions):
        joint_prims = stage.GetPrimAtPath(joint_path)
        if not joint_prims.IsValid():
            raise ValueError(f"Joint not found at {joint_path}, please check the path!")
        
        
        angular_drive = UsdPhysics.DriveAPI.Get(joint_prims, "angular")
        angular_drive.CreateTargetPositionAttr(float(position)/3.14*180)  # Ensure float for USD compatibility
        
        print(f"Set {joint_path} to position: {position/3.14*180}")
    return

def main():
    # Get assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find assets root path")
        simulation_app.close()
        sys.exit()

    # Open USD file
    usd_path = "/home/ime-lab/isaacsim/standalone_examples/scripts/robotenv.usd"
    if not open_stage(usd_path=usd_path):
        carb.log_error(f"Could not open USD file at {usd_path}")
        simulation_app.close()
        sys.exit()

    # Create simulation world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    # Set up camera and light
    stage = omni.usd.get_context().get_stage()
    camera_path = "/World/Camera"
    if not stage.GetPrimAtPath(camera_path).IsValid():
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.AddTranslateOp().Set(Gf.Vec3f(0, 0, 2))

    camera_light_path = "/World/Camera/CameraLight"
    if not stage.GetPrimAtPath(camera_light_path).IsValid():
        camera_light = UsdLux.DistantLight.Define(stage, camera_light_path)
        camera_light.CreateIntensityAttr(5000)
        camera_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    # Initialize PyBullet in DIRECT mode (no GUI)
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    plane_id = p.loadURDF("plane.urdf")

    # Load robot in PyBullet
    start_pos = [0.03, 0, 1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF("/home/ime-lab/XJQR/twoleg_foot/urdf/twoleg_foot.urdf", 
                         start_pos, start_orientation, useFixedBase=True)

    # Joint indices for left and right legs
    left_leg_joint_indices = [0, 1, 2, 3, 4, 5]
    right_leg_joint_indices = [6, 7, 8, 9, 10, 11]
    left_leg_end_link_index = 5
    right_leg_end_link_index = 11

    # Create leg planners
    left_leg_planner = LegTrajectoryPlanner(robot_id, left_leg_joint_indices, 
                                          left_leg_end_link_index, "Left Leg")
    right_leg_planner = LegTrajectoryPlanner(robot_id, right_leg_joint_indices, 
                                           right_leg_end_link_index, "Right Leg")

    # Target positions and orientation
    target_pos_left = [0.0, -0.2, 0.92]
    target_pos_right = [-0.0, -0.2, 0.92]
    target_quat = p.getQuaternionFromEuler([0, -90, 0])
    duration = 2.0
    steps = 100

    # Compute trajectories
    start_q_left = left_leg_planner.getCurrentJointValues()
    traj_left, t_list = left_leg_planner.getTrajectory(start_q_left, target_pos_left, 
                                                     target_quat, duration, steps)
    
    start_q_right = right_leg_planner.getCurrentJointValues()
    traj_right, _ = right_leg_planner.getTrajectory(start_q_right, target_pos_right, 
                                                  target_quat, duration, steps)

    # Define joint paths in Isaac Sim
    leftleg_prim_path = [
        "/twoleg_foot/joints/Left_Joint1",
        "/twoleg_foot/joints/Left_Joint2",
        "/twoleg_foot/joints/Left_Joint3",
        "/twoleg_foot/joints/Left_Joint4",
        "/twoleg_foot/joints/Left_Joint5",
        "/twoleg_foot/joints/Left_Joint6"
    ]
    rightleg_prim_path = [
        "/twoleg_foot/joints/Right_Joint1",
        "/twoleg_foot/joints/Right_Joint2",
        "/twoleg_foot/joints/Right_Joint3",
        "/twoleg_foot/joints/Right_Joint4",
        "/twoleg_foot/joints/Right_Joint5",
        "/twoleg_foot/joints/Right_Joint6"
    ]

    # Simulation loop
    my_world.reset()
    step_index = 0
    reset_needed = False

    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_stopped() and not reset_needed:
            reset_needed = True

        if my_world.is_playing():
            if reset_needed:
                my_world.reset()
                reset_needed = False
                step_index = 0

            # Apply joint angles from trajectories
            if step_index < len(traj_left):
                leg_joint_control(leftleg_prim_path, traj_left[step_index])
                leg_joint_control(rightleg_prim_path, traj_right[step_index])
                step_index += 1
            else:
                # Loop or stop after trajectory completes
                # step_index = 0  # Loop the trajectory
                simulation_app.update()

    # Cleanup
    p.disconnect()
    simulation_app.close()

if __name__ == "__main__":
    main()
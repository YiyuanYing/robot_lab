import numpy as np
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
    def __init__(self, joint_indices, leg_name="Leg"):
        self.joint_indices = joint_indices
        self.leg_name = leg_name
        self.lowerLimits = [-2.093, -2.093, -3.14, -2.093, -3.14, -3.14,
                           -2.093, -2.093, -3.14, -2.093, -3.14, -3.14]
        self.upperLimits = [2.093, 2.093, 3.14, 2.093, 3.14, 3.14,
                           2.093, 2.093, 3.14, 2.093, 3.14, 3.14]

    def getTrajectory(self, start_joint_value, target_joints, t, point_num):
        q0 = np.array(start_joint_value)
        qf = np.array(target_joints)

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

def leg_joint_control(leg_prim_path, target_positions, lock_joints=True):
    stage = omni.usd.get_context().get_stage()
    
    for joint_path, position in zip(leg_prim_path, target_positions):
        joint_prims = stage.GetPrimAtPath(joint_path)
        if not joint_prims.IsValid():
            raise ValueError(f"Joint not found at {joint_path}")
        
        angular_drive = UsdPhysics.DriveAPI.Get(joint_prims, "angular")
        if lock_joints:
            angular_drive.CreateTargetPositionAttr(float(position)/np.pi*180)
            angular_drive.CreateMaxForceAttr(1000000.0)  # Lock joint
        else:
            angular_drive.CreateMaxForceAttr(0.0)  # Unlock joint for free movement
        
        print(f"Set {joint_path} to position: {position/np.pi*180}")
    return

def add_fixed_joint(link_path, board_path, joint_name):
    stage = omni.usd.get_context().get_stage()
    joint = UsdPhysics.FixedJoint.Define(stage, f"/twoleg_foot/{joint_name}")
    joint.GetBody0Rel().SetTargets([link_path])
    joint.GetBody1Rel().SetTargets([board_path])
    return joint

def remove_fixed_joint(joint_name):
    stage = omni.usd.get_context().get_stage()
    joint_path = f"/World/{joint_name}"
    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)

def main():
    # Initialize simulation
    # simulation_app = SimulationApp({"headless": False})
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

    # Setup world and camera
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()
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

    # Setup leg planners
    left_leg_joint_indices = [0, 1, 2, 3, 4, 5]
    right_leg_joint_indices = [6, 7, 8, 9, 10, 11]

    left_leg_planner = LegTrajectoryPlanner(left_leg_joint_indices, "Left Leg")
    right_leg_planner = LegTrajectoryPlanner(right_leg_joint_indices, "Right Leg")

    # Joint paths
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

    # Target joint positions (converted from degrees to radians)
    target_joints = np.array([-30.7, 90, 73.4, 39.7, 5, -18.9]) * np.pi / 180
    duration = 5.0
    steps = 100

    # Simulation loop
    my_world.reset()
    step_index = 0
    state = "RIGHT_LEG_MOVING"  # States: RIGHT_LEG_MOVING, LEFT_LEG_MOVING
    right_fixed_joint = None
    left_fixed_joint = None
    start_q_left = [0.0] * 6  # Assume zero initial position
    start_q_right = [0.0] * 6

    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_stopped():
            continue

        if my_world.is_playing():
            if state == "RIGHT_LEG_MOVING":
                if step_index == 0:
                    # Compute right leg trajectory
                    traj_right, t_list = right_leg_planner.getTrajectory(
                        start_q_right, target_joints, duration, steps)
                    # Unlock left leg
                    leg_joint_control(leftleg_prim_path, [0]*6, lock_joints=False)

                if step_index < len(traj_right):
                    leg_joint_control(rightleg_prim_path, traj_right[step_index])
                    step_index += 1
                else:
                    # Add fixed joint for right leg
                    remove_fixed_joint("LeftFixedJoint")
                    right_fixed_joint = add_fixed_joint(
                        "/twoleg_foot/Right_Link6",
                        "/twoleg_foot/board",
                        "RightFixedJoint"
                    )
                    start_q_right = target_joints  # Update starting position
                    state = "LEFT_LEG_MOVING"
                    step_index = 0

            elif state == "LEFT_LEG_MOVING":
                if step_index == 0:
                    # Compute left leg trajectory
                    traj_left, t_list = left_leg_planner.getTrajectory(
                        start_q_left, target_joints, duration, steps)
                    # Unlock right leg
                    leg_joint_control(rightleg_prim_path, [0]*6, lock_joints=False)

                if step_index < len(traj_left):
                    leg_joint_control(leftleg_prim_path, traj_left[step_index])
                    step_index += 1
                else:
                    # Add fixed joint for left leg
                    remove_fixed_joint("RightFixedJoint")
                    left_fixed_joint = add_fixed_joint(
                        "/twoleg_foot/Left_Link6",
                        "/twoleg_foot/board",
                        "LeftFixedJoint"
                    )
                    start_q_left = target_joints  # Update starting position
                    state = "RIGHT_LEG_MOVING"
                    step_index = 0

    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
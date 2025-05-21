import time
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
import omni.usd
from pxr import UsdPhysics, UsdGeom, Gf

def leg_joint_control(leg_prim_path, target_positions):
    stage = omni.usd.get_context().get_stage()
    for joint_path, position in zip(leg_prim_path, target_positions):
        joint_prim = stage.GetPrimAtPath(joint_path)
        if not joint_prim.IsValid():
            raise ValueError(f"找不到关节 {joint_path}，请检查路径是否正确！")

        angular_drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
        angular_drive.CreateTargetPositionAttr(position)

        print(f"设置 {joint_path} -> {position}")
    print("关节控制完成\n")

if __name__ == "__main__":
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        print("无法找到资源路径")
        simulation_app.close()
        sys.exit()

    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    stage = omni.usd.get_context().get_stage()
    camera_path = "/World/Camera"
    if not stage.GetPrimAtPath(camera_path).IsValid():
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.AddTranslateOp().Set(Gf.Vec3f(0, 0, 2))

    asset_path = "/home/ime-lab/isaacsim/standalone_examples/scripts/robotenv.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/twoleg_foot")
    my_world.reset()

    leftleg_prim_path = [
        "/World/twoleg_foot/joints/Left_Joint1",
        "/World/twoleg_foot/joints/Left_Joint2",
        "/World/twoleg_foot/joints/Left_Joint3",
        "/World/twoleg_foot/joints/Left_Joint4",
        "/World/twoleg_foot/joints/Left_Joint5",
        "/World/twoleg_foot/joints/Left_Joint6"
    ]

    rightleg_prim_path = [
        "/World/twoleg_foot/joints/Right_Joint1",
        "/World/twoleg_foot/joints/Right_Joint2",
        "/World/twoleg_foot/joints/Right_Joint3",
        "/World/twoleg_foot/joints/Right_Joint4",
        "/World/twoleg_foot/joints/Right_Joint5",
        "/World/twoleg_foot/joints/Right_Joint6"
    ]

    # 四个动作序列：左1 -> 右1 -> 左2 -> 右2
    sequence = [
        ("left",  [-30.7, -92, -73.4, -39.7, -5, -18.9]),
        ("right", [-30.7,  92,  73.4,  39.7,  5,  18.9]),
        ("left",  [ 30.7,  92,  73.4,  39.7,  5,  18.9]),
        ("right", [ 30.7, -92, -73.4, -39.7, -5, -18.9])
    ]

    idx = 0
    reset_needed = False

    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_stopped() and not reset_needed:
            reset_needed = True

        if my_world.is_playing():
            if reset_needed:
                my_world.reset()
                reset_needed = False

            my_world.get_observations()

            # 每两秒执行一次新的动作序列
            name, target_pos = sequence[idx]
            if name == "left":
                leg_joint_control(leftleg_prim_path, target_pos)
            else:
                leg_joint_control(rightleg_prim_path, target_pos)

            idx = (idx + 1) % len(sequence)
            time.sleep(2)  # 等待两秒

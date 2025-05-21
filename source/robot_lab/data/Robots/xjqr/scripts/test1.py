from isaacsim import SimulationApp
# 初始化仿真引擎
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
import omni.kit

import carb

import sys

# 使用usd API控制Angular Drive
from pxr import UsdPhysics, Usd, UsdLux, UsdGeom, Gf
# from omni.isaac.core.utils.prims import get_prim_at_path
# from omni.isaac.core.utils.transformations import set_world_transform
import omni.usd

def leg_joint_control(leg_prim_path, target_positions):
    stage = omni.usd.get_context().get_stage()
    
    # 设置驱动参数
    # target_positions = target_positions
    # max_force = 50
    # stiffness = 0.0
    # damping = 0.0

    for joint_path, position in zip(leg_prim_path, target_positions):
        joint_prims = stage.GetPrimAtPath(joint_path)
        if not joint_prims.IsValid():
            raise ValueError(f"找不到关节 {leg_prim_path[i]}，请检查路径是否正确！")

        angular_drive = UsdPhysics.DriveAPI.Get(joint_prims, "angular")

        angular_drive.CreateTargetPositionAttr(position)
        # angular_drive.CreateMaxForceAttr(max_force)
        # angular_drive.CreateStiffnessAttr(stiffness)
        # angular_drive.CreateDampingAttr(damping)

        print(f"成功设置 {joint_path} 位置: {position}")
    print("关节控制指令已发送！")
    return 

if __name__ == "__main__":
    # 获取isaacsim资源路径
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("无法找到数字资源路径")
        simulation_app.close()
        sys.exit()

    # 创建仿真世界，并加载模型和环境
    my_world = World(stage_units_in_meters = 1.0)
    my_world.scene.add_default_ground_plane()
    
    # 设置camera light
    stage = omni.usd.get_context().get_stage()

    ## 创建 Camera 并启用 Camera Light
    camera_path = "/World/Camera"
    if not stage.GetPrimAtPath(camera_path).IsValid():
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.AddTranslateOp().Set(Gf.Vec3f(0, 0, 2))  # 调整摄像机位置

    # camera_light_path = "/World/Camera/CameraLight"
    # if not stage.GetPrimAtPath(camera_light_path).IsValid():
    #     camera_light = UsdLux.DistantLight.Define(stage, camera_light_path)
    #     camera_light.CreateIntensityAttr(5000)  # 设定光照强度
    #     camera_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))  # 白光

    # ground_prim = get_prim_at_path("/World/defaultGroundPlane")
    # set_world_transform(ground_prim, translation=(0, 0, -5))

    asset_path = "/home/ime-lab/isaacsim/standalone_examples/scripts/robotenv.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/twoleg_foot")
    my_world.reset()

    # 配置关节控制器的prim path
    leftleg_prim_path = []
    rightleg_prim_path = []


    leftleg_prim_path.append("/World/twoleg_foot/joints/Left_Joint1")
    leftleg_prim_path.append("/World/twoleg_foot/joints/Left_Joint2")
    leftleg_prim_path.append("/World/twoleg_foot/joints/Left_Joint3")
    leftleg_prim_path.append("/World/twoleg_foot/joints/Left_Joint4")
    leftleg_prim_path.append("/World/twoleg_foot/joints/Left_Joint5")
    leftleg_prim_path.append("/World/twoleg_foot/joints/Left_Joint6")

    rightleg_prim_path.append("/World/twoleg_foot/joints/Right_Joint1")
    rightleg_prim_path.append("/World/twoleg_foot/joints/Right_Joint2")
    rightleg_prim_path.append("/World/twoleg_foot/joints/Right_Joint3")
    rightleg_prim_path.append("/World/twoleg_foot/joints/Right_Joint4")
    rightleg_prim_path.append("/World/twoleg_foot/joints/Right_Joint5")
    rightleg_prim_path.append("/World/twoleg_foot/joints/Right_Joint6")

    # stage = omni.usd.get_context().get_stage()
    # parent_prim_path = "\twoleg_foot\board\board"
    # left_foot_prim_path = "\twoleg_foot\Left_Link6"
    # right_foot_prim_path = "\twoleg_foot\Right_Link6"
    # left_foot_joint_path = "\twoleg_foot\left_foot_joint"
    # right_foot_joint_path = "\twoleg_foot\right_foot_joint"
    # 进入仿真循环
    i = 0
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
            # 简单控制
            
            # right_target_positions = [-30.7, 92, 73.4, 39.7, 5, -18.9]
            right_target_positions = [30.7, -92, -73.4, -39.7, -5, -18.9]
            left_target_positions = [30.7, 92, 73.4, 39.7, 5, -18.9]
            # left_target_positions = [-30.7, -92, -73.4, -39.7 ,-5, -18.9]
            # left_target_positions = [0, 0, 0, 0, 0, 0]
            leg_joint_control(leftleg_prim_path, left_target_positions)
            # left_joint_prim = UsdPhysics.FixedJoint.Define(stage, left_foot_joint_path)
            # left_joint_prim.CreateBody0Rel().SetTargets([parent_prim_path])
            # left_joint_prim.CreateBody1Rel().SetTargets([left_foot_prim_path])
            leg_joint_control(rightleg_prim_path, right_target_positions)

    simulation_app.close()

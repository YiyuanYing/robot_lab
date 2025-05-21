# ik_leg_solver_sync.py

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})  # 设置 True 则不打开GUI

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.motion_generation.kinematics_interface import KinematicsSolver
from omni.isaac.core.articulations import Articulation
import numpy as np
import time

# 1. 加载你的USD文件
usd_path = "/home/ime-lab/isaacsim/standalone_examples/scripts/robotenv.usd"
open_stage(usd_path)

# 2. 创建World对象并添加机器人
world = World()
world.scene.add_default_ground_plane()
world.reset()

# ✅ 注意：替换成你的 robot 实际 prim path
robot_prim_path = "/twoleg_foot"  # 请确认 .usd 文件中机器人是挂在 /robot 路径下
robot = world.scene.add(Articulation(prim_path=robot_prim_path, name="twoleg_foot"))

# 等待一小段时间让物理世界加载完成
for _ in range(10):
    world.step(render=True)

# 3. 创建逆向运动学求解器
ik_solver = KinematicsSolver()

# 4. 设置左右腿末端的 frame 名称（你需要从 .usd 文件中确认真实名称）
left_foot_frame = "Left_Link6"     # 替换为你左腿末端实际frame名称
right_foot_frame = "Right_Link6"   # 替换为你右腿末端实际frame名称

# 5. 设置目标位姿（目标位置 + 目标朝向）
left_target_pos = np.array([0.15, 0.1, 1.3])     # 单位：米
right_target_pos = np.array([0.15, -0.1, 1.3])
left_target_rot = np.eye(3)                        # 单位方向
right_target_rot = np.eye(3)

# 6. 左腿逆向运动学
left_joint_pos, left_success = ik_solver.compute_inverse_kinematics(
    frame_name=left_foot_frame,
    target_positions=left_target_pos,
    target_orientation=left_target_rot,
    position_tolerance=1e-3,
    orientation_tolerance=1e-2
)
print("左腿IK求解成功：", left_success)
print("左腿关节角度：", left_joint_pos)

# 7. 右腿逆向运动学
right_joint_pos, right_success = ik_solver.compute_inverse_kinematics(
    frame_name=right_foot_frame,
    target_positions=right_target_pos,
    target_orientation=right_target_rot,
    position_tolerance=1e-3,
    orientation_tolerance=1e-2
)
print("右腿IK求解成功：", right_success)
print("右腿关节角度：", right_joint_pos)

# 8. 应用结果（可选）到 robot，并渲染结果
if left_success and right_success:
    full_joint_pos = np.zeros(len(ik_solver.get_joint_names()))
    
    # ✅ 根据你的机器人结构，插入左右腿 joint pos（这里只是示意，需要你确认顺序）
    full_joint_pos[:6] = left_joint_pos
    full_joint_pos[6:12] = right_joint_pos
    
    robot.set_joint_positions(full_joint_pos)
    world.step(render=True)
    time.sleep(1)

# 9. 保持一段时间看效果
print("运行结束，可视化窗口将保留5秒")
for _ in range(300):
    world.step(render=True)
    time.sleep(1 / 60)

# 10. 关闭
simulation_app.close()

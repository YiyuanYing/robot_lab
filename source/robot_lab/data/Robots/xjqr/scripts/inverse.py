import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_data
import time



class DebugAxes(object):
    """
    可视化基于base的坐标系, 红色x轴, 绿色y轴, 蓝色z轴
    常用于检查当前关节pose或者测量关键点的距离
    用法:
    goalPosition1 = DebugAxes()
    goalPosition1.update([0,0.19,0.15
                         ],[0,0,0,1])
    """
    def __init__(self):
        self.uids = [-1, -1, -1]

    def update(self, pos, orn):
        """
        Arguments:
        - pos: len=3, position in world frame
        - orn: len=4, quaternion (x, y, z, w), world frame
        """
        pos = np.asarray(pos).reshape(3)

        rot3x3 = R.from_quat(orn).as_matrix()
        axis_x, axis_y, axis_z = rot3x3.T
        self.uids[0] = p.addUserDebugLine(pos, pos + axis_x * 0.05, [1, 0, 0], replaceItemUniqueId=self.uids[0])
        self.uids[1] = p.addUserDebugLine(pos, pos + axis_y * 0.05, [0, 1, 0], replaceItemUniqueId=self.uids[1])
        self.uids[2] = p.addUserDebugLine(pos, pos + axis_z * 0.05, [0, 0, 1], replaceItemUniqueId=self.uids[2])



class LegTrajectoryPlanner:
    def __init__(self, robot_id, joint_indices, end_effector_link_index, leg_name="Leg"):
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.endEffectorLinkIndex = end_effector_link_index
        self.leg_name = leg_name
        # self.lowerLimits = [-2.093, -2.093, -3.14, -2.093, -3.14, -3.14,
        #                     -2.093, -2.093, -3.14, -2.093, -3.14, -3.14]
        # self.upperLimits = [2.093, 2.093, 3.14, 2.093, 3.14, 3.14,
        #                     2.093, 2.093, 3.14, 2.093, 3.14, 3.14]
        # self.jointRanges = [4.186, 4.186, 6.28, 4.186, 6.28, 6.28,
        #                    4.186, 4.186, 6.28, 4.186, 6.28, 6.28]
        self.lowerLimits = [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14,
                            -3.14, -3.14, -3.14, -3.14, -3.14, -3.14]
        self.upperLimits = [3.14, 3.14, 3.14, 3.14, 3.14, 3.14,
                            3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
        self.jointRanges = [6.28, 6.28, 6.28, 6.28, 6.28, 6.28,
                            6.28, 6.28, 6.28, 6.28, 6.28, 6.28]
        self.jointDamping = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001,
                             0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]
        self.restPoses = [0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0]
        '''
            0~5: Left_Joint1 ~ Left_Joint6
            6~11: Right_Joint1 ~ Right_Joint6
        '''

    def getCurrentJointValues(self):
        return [p.getJointState(self.robot_id, idx)[0] for idx in self.joint_indices]

    def calcIk(self, target_pos, target_quat):
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            self.endEffectorLinkIndex,
            targetPosition = target_pos,
            targetOrientation = target_quat,
            lowerLimits = self.lowerLimits,
            upperLimits = self.upperLimits,
            jointRanges = self.jointRanges,
            jointDamping = self.jointDamping,
            restPoses = self.restPoses
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

        # 插值矩阵
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

    def playTrajectory(self, trajectory, interval=0.01):
        for joint_angles in trajectory:
            for i, joint_index in enumerate(self.joint_indices):
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_index,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=joint_angles[i],
                    force=2000000
                )
                # p.resetJointState(
                #     bodyUniqueId = self.robot_id,
                #     jointIndex = joint_index,
                #     targetValue = joint_angles[i]
                # )
            p.stepSimulation()
            time.sleep(interval)

    def printTrajectoryTable(self, trajectory, time_list):
        print(f"\n[{self.leg_name}] Trajectory:")
        print(f"{'Time(s)':>8} | " + " | ".join([f"Joint{i+1}" for i in range(6)]))
        print("-" * 70)
        for i in range(len(trajectory)):
            time_str = f"{time_list[i]:>7.3f}"
            joint_str = " | ".join([f"{angle:>7.3f}" for angle in trajectory[i]])
            print(f"{time_str} | {joint_str}")


def main():
    # 启动 PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    plane_id = p.loadURDF("plane.urdf")

    # 加载机器人（你可换成自己的 URDF 路径）
    start_pos = [0.03, 0, 1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF("/home/ime-lab/XJQR/twoleg_foot/urdf/twoleg_foot.urdf", start_pos, start_orientation, useFixedBase=True)

    # 左右腿关节索引（根据你实际 URDF 修改）
    left_leg_joint_indices = [0, 1, 2, 3, 4, 5]
    right_leg_joint_indices = [6, 7, 8, 9, 10, 11]

    # 末端 link 索引（示例）
    left_leg_end_link_index = 5
    right_leg_end_link_index = 11

    # 创建两个腿的 planner
    left_leg_planner = LegTrajectoryPlanner(robot_id, left_leg_joint_indices, left_leg_end_link_index, "Left Leg")
    right_leg_planner = LegTrajectoryPlanner(robot_id, right_leg_joint_indices, right_leg_end_link_index, "Right Leg")

    # 设置目标姿态（左右腿目标不同位置避免重叠）
    target_pos_left = [0.01, -0.25, 0.9]
    target_pos_right = [-0.01, 0.2, 0.9]
    target_quat = p.getQuaternionFromEuler([0, -90, 0])

    # 时长、点数
    duration = 2.0
    steps = 100

    # 左腿轨迹
    start_q_left = left_leg_planner.getCurrentJointValues()
    traj_left, t_list = left_leg_planner.getTrajectory(start_q_left, target_pos_left, target_quat, duration, steps)
    left_leg_planner.printTrajectoryTable(traj_left, t_list)

    # 右腿轨迹
    start_q_right = right_leg_planner.getCurrentJointValues()
    traj_right, _ = right_leg_planner.getTrajectory(start_q_right, target_pos_right, target_quat, duration, steps)
    right_leg_planner.printTrajectoryTable(traj_right, t_list)

    # 播放轨迹（可选择只播放一条）
    print("\n播放左腿轨迹...")
    left_leg_planner.playTrajectory(traj_left, interval=duration/steps)
    goalPosition1 = DebugAxes()
    goalPosition1.update(target_pos_left,target_quat)

    print("\n播放右腿轨迹...")
    right_leg_planner.playTrajectory(traj_right, interval=duration/steps)
    goalPosition2 = DebugAxes()
    goalPosition2.update(target_pos_right,target_quat)

    # 保持窗口
    while True:
        p.stepSimulation()
        time.sleep(0.01)


if __name__ == "__main__":
    main()

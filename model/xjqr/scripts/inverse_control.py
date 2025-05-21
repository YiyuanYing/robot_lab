import numpy as np
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
        self.lowerLimits = [-2.093, -2.093, -3.14, -2.093, -3.14, -3.14,
                            -2.093, -2.093, -3.14, -2.093, -3.14, -3.14]
        self.upperLimits = [2.093, 2.093, 3.14, 2.093, 3.14, 3.14,
                            2.093, 2.093, 3.14, 2.093, 3.14, 3.14]
        self.jointRanges = [4.186, 4.186, 6.28, 4.186, 6.28, 6.28,
                           4.186, 4.186, 6.28, 4.186, 6.28, 6.28]
        self.jointDamping = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001,
                             0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]
        self.restPoses = [0, 0, 0, 0, 0, 0, 
                          0, 0, 0, 0, 0, 0]

    def getCurrentJointValues(self):
        return [p.getJointState(self.robot_id, idx)[0] for idx in self.joint_indices]

    def calcIk(self, target_pos, target_quat):
        joint_poses = p.calculateInverseKinematics(
            bodyUniqueId = self.robot_id,
            endEffectorLinkIndex = self.endEffectorLinkIndex,
            targetPosition=target_pos,
            targetOrientation=target_quat,
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
                    force=200
                )
            for i in range(10):
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

    # 加载机器人（你可以换成自己的 URDF 路径）
    start_pos = [0, 0, 1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF("/home/ime-lab/XJQR/twoleg_foot/urdf/twoleg_foot.urdf", start_pos, start_orientation, useFixedBase=True)

    # 左右腿关节索引
    left_leg_joint_indices = [0, 1, 2, 3, 4, 5]
    right_leg_joint_indices = [6, 7, 8, 9, 10, 11]

    # 足端链接索引
    left_leg_end_link_index = 5
    right_leg_end_link_index = 11

    # 创建 planner
    left_leg_planner = LegTrajectoryPlanner(robot_id, left_leg_joint_indices, left_leg_end_link_index, "Left Leg")
    right_leg_planner = LegTrajectoryPlanner(robot_id, right_leg_joint_indices, right_leg_end_link_index, "Right Leg")

    duration = 5.0
    steps = 400

    while True:
        print("\n选择操作：")
        print("1. 控制左腿末端")
        print("2. 控制右腿末端")
        print("3. 查看当前左腿足端位置与姿态")
        print("4. 查看当前右腿足端位置与姿态")
        print("q. 退出")
        choice = input("请输入选项：")

        if choice == '1' or choice == '2':
            x = float(input("请输入目标 x 坐标: "))
            y = float(input("请输入目标 y 坐标: "))
            z = float(input("请输入目标 z 坐标: "))
            roll = float(input("请输入目标 roll（角度）: "))
            pitch = float(input("请输入目标 pitch（角度）: "))
            yaw = float(input("请输入目标 yaw（角度）: "))
            target_pos = [x, y, z]
            target_quat = p.getQuaternionFromEuler([np.radians(roll), np.radians(pitch), np.radians(yaw)])

            if choice == '1':
                start_q = left_leg_planner.getCurrentJointValues()
                traj, t_list = left_leg_planner.getTrajectory(start_q, target_pos, target_quat, duration, steps)
                left_leg_planner.printTrajectoryTable(traj, t_list)
                left_leg_planner.playTrajectory(traj, interval=duration / steps)
            else:
                start_q = right_leg_planner.getCurrentJointValues()
                traj, t_list = right_leg_planner.getTrajectory(start_q, target_pos, target_quat, duration, steps)
                right_leg_planner.printTrajectoryTable(traj, t_list)
                right_leg_planner.playTrajectory(traj, interval=duration / steps)

        elif choice == '3':
            state = p.getLinkState(robot_id, left_leg_end_link_index)
            pos = state[4]
            orn = state[5]
            rpy = p.getEulerFromQuaternion(orn)
            print("\n当前左腿足端位置与姿态：")
            print(f"位置: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
            print(f"姿态 (欧拉角): roll={np.degrees(rpy[0]):.1f}°, pitch={np.degrees(rpy[1]):.1f}°, yaw={np.degrees(rpy[2]):.1f}°")

        elif choice == '4':
            state = p.getLinkState(robot_id, right_leg_end_link_index)
            pos = state[4]
            orn = state[5]
            rpy = p.getEulerFromQuaternion(orn)
            print("\n当前右腿足端位置与姿态：")
            print(f"位置: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
            print(f"姿态 (欧拉角): roll={np.degrees(rpy[0]):.1f}°, pitch={np.degrees(rpy[1]):.1f}°, yaw={np.degrees(rpy[2]):.1f}°")

        elif choice.lower() == 'q':
            print("退出程序。")
            break

        else:
            print("无效选项，请重新输入。")

        p.stepSimulation()
        time.sleep(0.01)


if __name__ == "__main__":
    main()

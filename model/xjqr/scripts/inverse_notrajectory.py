import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_data
import time


class DebugAxes(object):
    def __init__(self):
        self.uids = [-1, -1, -1]

    def update(self, pos, orn):
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
        self.lowerLimits = [-2.093] * 12
        self.upperLimits = [2.093] * 12
        self.jointRanges = [4.186] * 12
        self.jointDamping = [0.0001] * 12
        self.restPoses = [0.0] * 12

    def getCurrentJointValues(self):
        return [p.getJointState(self.robot_id, idx)[0] for idx in self.joint_indices]

    def calcIk(self, target_pos, target_quat):
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            self.endEffectorLinkIndex,
            targetPosition=target_pos,
            targetOrientation=target_quat,
            lowerLimits=self.lowerLimits,
            upperLimits=self.upperLimits,
            jointRanges=self.jointRanges,
            jointDamping=self.jointDamping,
            restPoses=self.restPoses
        )
        return [joint_poses[i] for i in self.joint_indices]

    def moveToTarget(self, target_pos, target_quat):
        joint_angles = self.calcIk(target_pos, target_quat)
        print(f"\n[{self.leg_name}] Target Joint Angles:")
        for i, angle in enumerate(joint_angles):
            print(f"Joint {i+1}: {angle:.3f}")
        for i, joint_index in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_index, joint_angles[i])
        p.stepSimulation()


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    start_pos = [0, 0, 1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF("/home/ime-lab/XJQR/twoleg_foot/urdf/twoleg_foot.urdf",
                          start_pos, start_orientation, useFixedBase=True)

    left_leg_joint_indices = [0, 1, 2, 3, 4, 5]
    right_leg_joint_indices = [6, 7, 8, 9, 10, 11]

    left_leg_end_link_index = 5
    right_leg_end_link_index = 11

    left_leg_planner = LegTrajectoryPlanner(robot_id, left_leg_joint_indices, left_leg_end_link_index, "Left Leg")
    right_leg_planner = LegTrajectoryPlanner(robot_id, right_leg_joint_indices, right_leg_end_link_index, "Right Leg")

    target_pos_left = [0.05, -0.25, 0.9]
    target_pos_right = [-0.05, 0.25, 0.9]
    target_quat = p.getQuaternionFromEuler([0, -90, 0])

    print("\n左腿移动到目标位置...")
    left_leg_planner.moveToTarget(target_pos_left, target_quat)
    goalPosition1 = DebugAxes()
    goalPosition1.update(target_pos_left, target_quat)

    time.sleep(1)

    print("\n右腿移动到目标位置...")
    right_leg_planner.moveToTarget(target_pos_right, target_quat)
    goalPosition2 = DebugAxes()
    goalPosition2.update(target_pos_right,target_quat)

    while True:
        p.stepSimulation()
        time.sleep(0.01)


if __name__ == "__main__":
    main()

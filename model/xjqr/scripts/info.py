import pybullet as p
import pybullet_data
import time

def print_joint_and_link_info(robot_id):
    num_joints = p.getNumJoints(robot_id)
    print(f"机器人共有 {num_joints} 个关节。\n")

    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_index = joint_info[0]
        joint_name = joint_info[1].decode('utf-8')
        link_name = joint_info[12].decode('utf-8')
        # print(f"Joint Index: {joint_index:>2} | Joint Name: {joint_name:<20} | Link Name: {link_name}")
        print(joint_info)

def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    # 加载你自己的机器人URDF文件，如果是自定义路径，请修改这行
    robot_path = "/home/ime-lab/XJQR/twoleg_foot/urdf/twoleg_foot.urdf"  # 替换为你自己的URDF路径
    robot_start_pos = [0, 0, 1]
    robot_start_ori = p.getQuaternionFromEuler([0, 0, 0])

    robot_id = p.loadURDF(robot_path, robot_start_pos, robot_start_ori, useFixedBase=True)

    # 打印关节和链接信息
    print_joint_and_link_info(robot_id)

    # 让窗口保持一会看信息
    time.sleep(50000)
    p.disconnect()

if __name__ == "__main__":
    main()

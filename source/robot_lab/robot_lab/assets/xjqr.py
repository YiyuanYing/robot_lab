import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg

from robot_lab.assets import ISAACLAB_ASSETS_DATA_DIR

XJQR_2LEG_WHEEL=ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path=f"/home/ime-lab/XJQR/twoleg_foot/urdf/twoleg_foot/twoleg_foot.usd",
        usd_path=f"/home/ime-lab/isaacsim/standalone_examples/scripts/robotenv8.usd", 
        # usd_path需要根据文件实际目录更改，4090上放在isaacsim的文件夹下
        # repo的models都放在"source/robot_lab/data/Robots/xjqr"这里，比如"source/robot_lab/data/Robots/xjqr/robotenv8.usd"，需要根据电脑上的实际目录进行更改，建议使用绝对路径
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        #初始位置和初始姿态，初始姿态我设置成了前后扒拉在帆板上的角度（需要弧度制），关节名可以用正则表达式
        pos=(0.0, 0.0, 1.4),
        # pos = (0.0, 0.0, 0.22),
        joint_pos={
            "Left_Joint1": 0.0,
            "Left_Joint2": -90.0*3.14/180,
            "Left_Joint3": 45.0*3.14/180,
            "Left_Joint4": 45.0*3.14/180,
            "Left_Joint5": 90.0*3.14/180,
            "Left_Joint6": 1.0,
            "Right_Joint1": 0.0,
            "Right_Joint2": 90.0*3.14/180,
            "Right_Joint3": -45.0*3.14/180,
            "Right_Joint4": -45.0*3.14/180,
            "Right_Joint5": 90.0*3.14/180,
            "Right_Joint6": 1.0
            # "Left_Joint.*": 0.0,
            # "Right_Joint.*":0.0,
        },
        joint_vel={
            "Left_Joint.*": 0.0,              # 匹配 Left_Joint1 到 Left_Joint6
            "Right_Joint.*": 0.0,             # 匹配 Right_Joint1 到 Right_Joint6
            # "Left_Wheel.*_Joint": 0.0,       # 匹配左轮子关节
            # "Right_Wheel.*_Joint": 0.0,      # 匹配右轮子关节
        },
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs_joint1": DCMotorCfg(
            joint_names_expr=[".*Joint1"],
            effort_limit=9.0,
            saturation_effort=9.0,
            velocity_limit=30.0,
            stiffness=25.0,
            damping=0.5,
            friction=0.0,
        ),
        "legs_joint2": DCMotorCfg(
            joint_names_expr=[".*Joint2"],
            effort_limit=9.0,
            saturation_effort=9.0,
            velocity_limit=30.0,
            stiffness=25.0,
            damping=0.5,
            friction=0.0,
        ),
        "legs_joint3": DCMotorCfg(
            joint_names_expr=[".*Joint3"],
            effort_limit=1.0,
            saturation_effort=1.0,
            velocity_limit=30.0,
            stiffness=25.0,
            damping=0.5,
            friction=0.0,
        ),
        "legs_joint4": DCMotorCfg(
            joint_names_expr=[".*Joint4"],
            effort_limit=3.0,
            saturation_effort=3.0,
            velocity_limit=30.0,
            stiffness=25.0,
            damping=0.5,
            friction=0.0,
        ),
        "legs_joint5": DCMotorCfg(
            joint_names_expr=[".*Joint5"],
            effort_limit=1.0,
            saturation_effort=1.0,
            velocity_limit=30.0,
            stiffness=25.0,
            damping=0.5,
            friction=0.0,
        ),
        "legs_joint6": DCMotorCfg(
            joint_names_expr=[".*Joint6"],
            effort_limit=1.0,
            saturation_effort=1.0,
            velocity_limit=30.0,
            stiffness=25.0,
            damping=0.5,
            friction=0.0,
        ),
        # "wheels": DCMotorCfg(
        #     joint_names_expr=[".*Wheel.*"],
        #     effort_limit=1.0,
        #     saturation_effort=1.0,
        #     velocity_limit=30.0,
        #     stiffness=0.0,
        #     damping=0.5,
        #     friction=0.0,
        # ),
    },
)
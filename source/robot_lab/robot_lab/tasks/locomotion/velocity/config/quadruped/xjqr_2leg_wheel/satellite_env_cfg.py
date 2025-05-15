from isaaclab.utils import configclass
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg


from robot_lab.tasks.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

##
# Pre-defined configs
##
# # use cloud assets
# from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # isort: skip
# use local assets
# from robot_lab.assets.unitree import UNITREE_GO2_CFG  # isort: skip

# import robot_lab.tasks.locomotion.velocity.mdp as mdp
from robot_lab.tasks.locomotion.velocity.velocity_env_cfg import ActionsCfg, LocomotionVelocityRoughEnvCfg, RewardsCfg

from robot_lab.assets.xjqr import XJQR_2LEG_WHEEL


@configclass
class xjqr2legWheelRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    base_link_name = "base_link"
    foot_link_name = ".*Link6"
    joint_names = [
        "Left_Joint1", "Left_Joint2", "Left_Joint3", "Left_Joint4", "Left_Joint5", "Left_Joint6",
        "Right_Joint1", "Right_Joint2", "Right_Joint3", "Right_Joint4", "Right_Joint5", "Right_Joint6",
    ]
    # joint_names = [
    #     "Left_Joint1", "Left_Joint2", "Left_Joint3", "Left_Joint4", "Left_Joint5",
    #     "Right_Joint1", "Right_Joint2", "Right_Joint3", "Right_Joint4", "Right_Joint5",
    # ]
    
    def __post_init__(self):
        super().__post_init__()

        
        # Scene
        # switch robot to xjqr_2leg_wheel
        self.scene.robot = XJQR_2LEG_WHEEL.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name
        self.scene.height_scanner_base.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name
        
        # scale down the terrains because the robot is small
        # self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.0, 0.0)
        # self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        # self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01
        # self.scene.terrain.terrain_type = "plane"
        # self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None
        
        # # Load customo environment from USD file
        # custom_env_path = "/home/ime-lab/isaacsim/test/satellite.usdc"
        # self.scene.terrain = None
        # add_reference_to_stage(usd_path=custom_env_path, prim_path="/World/envExp")
        # self.scene.env_prim_path = "/World/envExp"
        
        # ------------------------------Observations------------------------------
        # self.observations.policy.joint_pos.func = mdp.joint_pos_rel_without_wheel
        # self.observations.policy.joint_pos.params["wheel_asset_cfg"] = SceneEntityCfg(
        #     "robot", joint_names=[self.foot_link_name]
        # )
        # self.observations.critic.joint_pos.func = mdp.joint_pos_rel_without_wheel
        # self.observations.critic.joint_pos.params["wheel_asset_cfg"] = SceneEntityCfg(
        #     "robot", joint_names=[self.foot_link_name]
        # )
        
        self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05
        self.observations.policy.base_lin_vel = None
        self.observations.policy.height_scan = None
        self.observations.policy.joint_pos.params["asset_cfg"].joint_names = self.joint_names
        self.observations.policy.joint_vel.params["asset_cfg"].joint_names = self.joint_names

        # ------------------------------Actions------------------------------
        # reduce action scale
        self.actions.joint_pos.scale = {".*_Joint1": 0.125, "^(?!.*_Joint1).*": 0.25}
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}
        self.actions.joint_pos.joint_names = self.joint_names

        # ------------------------------Events------------------------------
        # self.events.randomize_reset_base.params = {
        #     "pose_range": {
        #         "x": (-0.5, 0.5),
        #         "y": (-0.5, 0.5),
        #         "z": (0.0, 0.2),
        #         "roll": (-3.14, 3.14),
        #         "pitch": (-3.14, 3.14),
        #         "yaw": (-3.14, 3.14),
        #     },
        #     "velocity_range": {
        #         "x": (-0.5, 0.5),
        #         "y": (-0.5, 0.5),
        #         "z": (-0.5, 0.5),
        #         "roll": (-0.5, 0.5),
        #         "pitch": (-0.5, 0.5),
        #         "yaw": (-0.5, 0.5),
        #     },
        # }
        self.events.randomize_reset_base.params = {
            "pose_range": {
                "x": (-0.015, 0.015),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.02),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        }
        self.events.randomize_rigid_body_mass.params["asset_cfg"].body_names = [self.base_link_name]
        self.events.randomize_com_positions.params["asset_cfg"].body_names = [self.base_link_name]
        self.events.randomize_apply_external_force_torque.params["asset_cfg"].body_names = [self.base_link_name]

        # ------------------------------Rewards------------------------------
        # General
        self.rewards.is_terminated.weight = 0

        # Root penalties
        self.rewards.lin_vel_z_l2.weight = -2.0

        # self.rewards.track_lin_vel_y_exp.func = "robot_lab.tasks.locomotion.velocity.rewards.track_lin_vel_y_exp"
        # self.rewards.track_lin_vel_y_exp.weight = 5.0
        # self.rewards.track_lin_vel_y_exp.params = {
        #     "std": 0.25,
        #     "command_name": "base_velocity",
        # }
        self.rewards.ang_vel_xy_l2.weight = -0.5
        self.rewards.flat_orientation_l2.weight = -0.001
        self.rewards.base_height_l2.weight = -1.0
        self.rewards.base_height_l2.params["target_height"] = 1.39
        self.rewards.base_height_l2.params["asset_cfg"].body_names = [self.base_link_name]
        self.rewards.body_lin_acc_l2.weight = 0.0
        self.rewards.body_lin_acc_l2.params["asset_cfg"].body_names = [self.base_link_name]

        # Joint penaltie
        self.rewards.joint_torques_l2.weight = -2.5e-5
        self.rewards.joint_vel_l2.weight = 0
        self.rewards.joint_acc_l2.weight = -2.5e-7
        # self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_hip_l1", -0.2, [".*_hip_joint"])
        self.rewards.joint_pos_limits.weight = -1.0
        self.rewards.joint_vel_limits.weight = 0
        self.rewards.joint_power.weight = -2e-5
        self.rewards.stand_still_without_cmd.weight = -2.0
        self.rewards.joint_pos_penalty.weight = -1.0
        # self.rewards.joint_mirror.weight = -0.05
        # self.rewards.joint_mirror.params["mirror_joints"] = [
        #     # ["FR_(hip|thigh|calf).*", "RL_(hip|thigh|calf).*"],
        #     # ["FL_(hip|thigh|calf).*", "RR_(hip|thigh|calf).*"],
        #     ["Left_Joint.*", "Right_Joint.*"],
        # ]

        # Action penalties
        self.rewards.action_rate_l2.weight = -0.01

        # Contact sensor
        self.rewards.undesired_contacts.weight = -1.0
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]
        self.rewards.contact_forces.weight = -1.5e-4
        self.rewards.contact_forces.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]

        # Velocity-tracking rewards
        self.rewards.track_lin_vel_xy_exp.weight = 8.0 #zhiqian 3.0
        self.rewards.track_ang_vel_z_exp.weight = 0.0

        # Others
        self.rewards.feet_air_time.weight = 0.0
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_contact.weight = 5.0
        # self.rewards.feet_contact.params["expect_contact_num"] = 1
        self.rewards.feet_contact.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_contact_without_cmd.weight = -0.2
        self.rewards.feet_contact_without_cmd.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_stumble.weight = -0.02
        self.rewards.feet_stumble.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_slide.weight = -0.03
        self.rewards.feet_slide.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_slide.params["asset_cfg"].body_names = [self.foot_link_name]
        # self.rewards.feet_height.weight = 0
        # self.rewards.feet_height.params["target_height"] = 0.05
        # self.rewards.feet_height.params["asset_cfg"].body_names = [self.foot_link_name]
        # self.rewards.feet_height_body.weight = -5.0
        # self.rewards.feet_height_body.params["target_height"] = -0.2
        # self.rewards.feet_height_body.params["asset_cfg"].body_names = [self.foot_link_name]
        # self.rewards.feet_gait.weight = 0
        # self.rewards.feet_gait.params["synced_feet_pair_names"] = (("FL_foot", "RR_foot"), ("FR_foot", "RL_foot"))
        self.rewards.upward.weight = 1.0

        # If the weight of rewards is 0, set rewards to None
        if self.__class__.__name__ == "xjqr2legWheelRoughEnvCfg":
            self.disable_zero_weight_rewards()

        # ------------------------------Terminations------------------------------
        self.terminations.illegal_contact.params["sensor_cfg"].body_names = [self.base_link_name]
        # self.terminations.illegal_contact = None

        # ------------------------------Commands------------------------------
        # self.commands.base_velocity.ranges.lin_vel_x = (-1.5, 1.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-1.5, 1.5)
        # self.commands.base_velocity.ranges.ang_vel_z = (-1.5, 1.5)

        
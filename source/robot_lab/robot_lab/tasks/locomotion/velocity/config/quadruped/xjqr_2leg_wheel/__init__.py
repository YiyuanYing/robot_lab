import gymnasium as gym

from . import agents
from .satellite_env_cfg import xjqr2legWheelRoughEnvCfg

##
# Register Gym environments.
##

# gym.register(
#     id="RobotLab-Isaac-Velocity-Flat-Unitree-Go2-v0",
#     entry_point="isaaclab.envs:ManagerBasedRLEnv",
#     disable_env_checker=True,
#     kwargs={
#         "env_cfg_entry_point": f"{__name__}.flat_env_cfg:UnitreeGo2FlatEnvCfg",
#         "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:UnitreeGo2FlatPPORunnerCfg",
#     },
# )

# gym.register(
#     id="RobotLab-Isaac-Velocity-Rough-Unitree-Go2-v0",
#     entry_point="isaaclab.envs:ManagerBasedRLEnv",
#     disable_env_checker=True,
#     kwargs={
#         "env_cfg_entry_point": f"{__name__}.rough_env_cfg:UnitreeGo2RoughEnvCfg",
#         "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:UnitreeGo2RoughPPORunnerCfg",
#     },
# )


# 注册机器人的id和其他函数名
gym.register(
    id="Isaac-Velocity-Satellite-XJQR-2leg-wheel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=False,
    kwargs={
        # "env_cfg_entry_point": f"{__name__}.rough_env_cfg:xjqr2legWheelRoughEnvCfg",
        "env_cfg_entry_point": xjqr2legWheelRoughEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:xjqr2legWheelRoughPPORunnerCfg",
    },
)
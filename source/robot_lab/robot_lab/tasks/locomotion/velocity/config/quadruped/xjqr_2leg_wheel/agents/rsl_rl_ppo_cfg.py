from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class xjqr2legWheelRoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    # 这里的项目都可以在isaaclab的相关文件中找到函数定义
    num_steps_per_env = 24
    max_iterations = 20000
    save_interval = 100
    experiment_name = "xjqr_2leg_Wheel_Rough"
    empirical_normalization = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[256, 256, 256],
        critic_hidden_dims=[256, 256, 256],
        activation="elu",
        noise_std_type="log",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-2, # 学习率，默认的是1e-3，之前改成了1e-2
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
        normalize_advantage_per_mini_batch=True,
    )


@configclass
class xjqr2legWheelFlatPPORunnerCfg(xjqr2legWheelRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 5000
        self.experiment_name = "xjqr_2leg_Wheel_Rough"

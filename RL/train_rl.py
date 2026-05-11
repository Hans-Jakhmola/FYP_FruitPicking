import os
import numpy as np

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    BaseCallback, CallbackList, CheckpointCallback, EvalCallback,
)
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.noise import NormalActionNoise

from ur5_pickplace import UR5PickPlaceEnv


# Unique name for this training run.
# All models, checkpoints and TensorBoard logs are saved under this tag.
RUN_TAG = "pick_place_sac_v9F"


def make_env(gui=False):
    # Wrap the custom PyBullet environment with Monitor.
    # Monitor records episode rewards, episode lengths and evaluation statistics.
    return Monitor(UR5PickPlaceEnv(gui=gui))


class InfoLogCallback(BaseCallback):
    # Callback for logging useful environment info values to TensorBoard.
    # The environment returns these values in the info dictionary after each step.
    def __init__(self, keys, log_freq=1000, verbose=0):
        super().__init__(verbose)

        # List of info dictionary keys to track.
        self.keys = keys

        # Number of training steps between TensorBoard logging updates.
        self.log_freq = log_freq

        # Temporary storage for values collected between logging updates.
        self.buffer = {k: [] for k in keys}

    def _on_step(self):
        # Read the info dictionary returned by the environment.
        # For each selected key, store the value so an average can be logged.
        for info in self.locals.get("infos", []):
            for k in self.keys:
                if k in info:
                    self.buffer[k].append(float(info[k]))

        # Every log_freq steps, write the mean value of each tracked term.
        # This helps show whether the policy is grasping, lifting and moving to the drop point.
        if self.num_timesteps % self.log_freq == 0:
            for k, vals in self.buffer.items():
                if vals:
                    self.logger.record(f"task/{k}", float(np.mean(vals)))
                    self.buffer[k] = []

        # Returning True tells Stable-Baselines3 to continue training.
        return True


def main():
    # Create output folders for this run.
    model_dir = os.path.join("models", RUN_TAG)
    log_dir = os.path.join("logs", RUN_TAG)
    checkpoint_dir = os.path.join(model_dir, "checkpoints")
    best_model_dir = os.path.join(model_dir, "best_model")
    tb_log_dir = os.path.join(log_dir, "tensorboard")

    # Make sure all required output folders exist before training starts.
    for d in (model_dir, checkpoint_dir, best_model_dir, tb_log_dir):
        os.makedirs(d, exist_ok=True)

    # Print the run locations so they are visible in the terminal.
    print(f"[pick-place SAC] Run tag : {RUN_TAG}")
    print(f"[pick-place SAC] Models  -> {model_dir}")
    print(f"[pick-place SAC] Logs    -> {log_dir}")

    # Check that the custom Gymnasium environment follows the expected API.
    check_env(UR5PickPlaceEnv(gui=False), warn=True)

    # Create separate environments for training and evaluation.
    # Evaluation is kept separate so the best model can be selected consistently.
    train_env = make_env()
    eval_env = make_env()

    # Combine checkpoint saving, periodic evaluation and custom info logging.
    callbacks = CallbackList([
        # Saves model checkpoints during training.
        # This prevents losing progress and allows older checkpoints to be tested later.
        CheckpointCallback(
            save_freq=25_000,
            save_path=checkpoint_dir,
            name_prefix=f"sac_{RUN_TAG}",
            save_replay_buffer=True,
        ),

        # Periodically evaluates the current policy on separate episodes.
        # The best-performing model is saved in best_model_dir.
        EvalCallback(
            eval_env,
            best_model_save_path=best_model_dir,
            log_path=os.path.join(log_dir, "eval"),
            eval_freq=10_000,
            n_eval_episodes=10,
            deterministic=True,
        ),

        # Logs task-specific terms from the environment info dictionary.
        # These values help diagnose whether SAC is learning the correct sequence.
        InfoLogCallback(
            keys=[
                "is_grasping", "is_success", "dist_to_pregrasp",
                "dist_to_fruit", "dist_to_drop", "fruit_z",
                "lift_amount", "align_steps", "grasp_steps",
                "hold_bonus", "grasp_bonus", "drop_reward",
                "ungrip_penalty", "in_transport_phase",
            ],
            log_freq=1000,
        ),
    ])

    # Add exploration noise to the continuous action space.
    # The first three values affect end-effector x/y/z movement.
    # The fourth value affects the gripper command, so it is larger to help SAC explore closing.
    action_noise = NormalActionNoise(
        mean=np.zeros(4),
        sigma=np.array([0.1, 0.1, 0.1, 0.5]),
    )

    # Create the SAC model.
    model = SAC(
        policy="MlpPolicy",
        env=train_env,

        # Optimisation and replay-buffer settings.
        learning_rate=1e-3,
        buffer_size=1_000_000,
        batch_size=256,

        # Discount factor. Slightly lower than 0.99 so near-term grasp/lift rewards matter strongly.
        gamma=0.95,

        # Soft target-network update rate.
        tau=0.005,

        # Train once after every environment step.
        train_freq=1,
        gradient_steps=1,

        # Number of random/exploratory steps before gradient updates begin.
        learning_starts=5_000,

        # Fixed entropy coefficient to maintain exploration pressure.
        ent_coef=0.1,

        # Extra action noise to encourage exploration, especially gripper closing.
        action_noise=action_noise,

        # Three-layer MLP policy/value networks.
        policy_kwargs=dict(net_arch=[256, 256, 256]),

        # Print training progress and save TensorBoard logs.
        verbose=1,
        tensorboard_log=tb_log_dir,
        device="auto",
    )

    # Start training.
    # The callback handles checkpoints, best-model saving and reward-term logging.
    model.learn(
        total_timesteps=200_000,
        callback=callbacks,
        log_interval=20,
        progress_bar=True,
        tb_log_name="SAC",
    )

    # Save the final model at the end of training.
    final_path = os.path.join(model_dir, f"sac_{RUN_TAG}")
    model.save(final_path)
    print(f"[pick-place SAC] Saved -> {final_path}.zip")

    # Close environments after training
    train_env.close()
    eval_env.close()


if __name__ == "__main__":
    main()

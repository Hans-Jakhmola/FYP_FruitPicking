import os
import numpy as np

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    BaseCallback, CallbackList, CheckpointCallback, EvalCallback,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.noise import NormalActionNoise

from ur5_pickplace import UR5PickPlaceEnv


# Curriculum stage definitions.
# Each stage increases the random cube spawn range around the fixed start point.
STAGES = {
    1: (0.05, "curriculum_stage_1_5cm"),
    2: (0.10, "curriculum_stage_2_10cm"),
}


def make_env(spawn_range: float, gui: bool = False):
    # Wrap the custom PyBullet environment with Monitor.
    # spawn_range controls how much the cube position is randomised during training.
    return Monitor(UR5PickPlaceEnv(gui=gui, spawn_range=spawn_range))


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
        # This helps show whether the policy is still grasping, lifting and reaching the drop area.
        if self.num_timesteps % self.log_freq == 0:
            for k, vals in self.buffer.items():
                if vals:
                    self.logger.record(f"task/{k}", float(np.mean(vals)))
                    self.buffer[k] = []

        return True


def main():
    # Select which curriculum stage to train.
    # Stage 1 uses a smaller spawn range, then later stages increase the difficulty.
    CURRENT_STAGE = 2

    # Model to continue training from.
    # This should usually be the final or best model from the previous curriculum stage.
    START_MODEL = "models/curriculum_stage_1_5cm_PickplaceD_RandS1/sac_curriculum_stage_1_5cm_PickplaceD_RandS1.zip"

    # Number of additional training steps for this stage.
    TOTAL_STEPS = 200_000

    # Extra label added to this stage's output folders.
    RUN_TAG = "PickplaceD_RandS2"

    # Stop early if the starting model cannot be found.
    if not os.path.exists(START_MODEL):
        raise FileNotFoundError(
            f"Model not found: {START_MODEL}\n"
            f"Update START_MODEL to point at your previous stage best model."
        )

    # Get the spawn range and default tag for this curriculum stage.
    spawn_range, default_tag = STAGES[CURRENT_STAGE]
    tag = f"{default_tag}_{RUN_TAG}" if RUN_TAG else default_tag

    # Print the stage settings so the run is clear in the terminal.
    print(f"\n{'='*60}")
    print(f"CURRICULUM STAGE {CURRENT_STAGE}")
    print(f"Spawn range : ±{spawn_range*100:.0f}cm")
    print(f"Start model : {START_MODEL}")
    print(f"Steps       : {TOTAL_STEPS:,}")
    print(f"Output tag  : {tag}")
    print(f"{'='*60}\n")

    # Create output folders for this curriculum stage.
    model_dir = os.path.join("models", tag)
    log_dir = os.path.join("logs", tag)
    checkpoint_dir = os.path.join(model_dir, "checkpoints")
    best_model_dir = os.path.join(model_dir, "best_model")
    tb_log_dir = os.path.join(log_dir, "tensorboard")

    # Make sure all required output folders exist before training starts.
    for d in (model_dir, checkpoint_dir, best_model_dir, tb_log_dir):
        os.makedirs(d, exist_ok=True)

    # Create separate training and evaluation environments using the same spawn range.
    train_env = make_env(spawn_range)
    eval_env = make_env(spawn_range)

    # Load the previous model and continue training it on the harder spawn range.
    model = SAC.load(START_MODEL, env=train_env, device="auto")

    # Update the TensorBoard log folder for the new stage.
    model.tensorboard_log = tb_log_dir

    # Keep exploration noise during continued training.
    # The first three values affect end-effector x/y/z movement.
    # The fourth value affects the gripper command, so it is larger to encourage closing.
    model.action_noise = NormalActionNoise(
        mean=np.zeros(4),
        sigma=np.array([0.1, 0.1, 0.1, 0.5]),
    )

    # Combine checkpoint saving, periodic evaluation and custom info logging.
    callbacks = CallbackList([
        # Saves model checkpoints during training.
        CheckpointCallback(
            save_freq=25_000,
            save_path=checkpoint_dir,
            name_prefix=f"sac_{tag}",
            save_replay_buffer=True,
        ),

        # Periodically evaluates the current policy on separate episodes.
        # The best-performing model is saved in best_model_dir.
        EvalCallback(
            eval_env,
            best_model_save_path=best_model_dir,
            log_path=os.path.join(log_dir, "eval"),
            eval_freq=10_000,
            n_eval_episodes=20,
            deterministic=True,
        ),

        # Logs task specific terms from the environment info dictionary.

        InfoLogCallback(
            keys=[
                "is_grasping",
                "is_success",
                "dist_to_drop",
                "lift_amount",
                "in_transport_phase",
            ],
            log_freq=1000,
        ),
    ])

    # Continue training the loaded SAC model.
    # reset_num_timesteps=False keeps the timestep count continuous from the previous stage.
    model.learn(
        total_timesteps=TOTAL_STEPS,
        callback=callbacks,
        log_interval=20,
        progress_bar=True,
        tb_log_name="SAC",
        reset_num_timesteps=False,
    )

    # Save the final model at the end
    final_path = os.path.join(model_dir, f"sac_{tag}")
    model.save(final_path)
    print(f"\n[stage {CURRENT_STAGE}] Saved -> {final_path}.zip")

    # Print the location of the best model and instructions for the next curriculum stage.
    best_path = os.path.join(best_model_dir, "best_model.zip")
    print(f"\n{'='*60}")
    print(f"STAGE {CURRENT_STAGE} COMPLETE")
    print(f"{'='*60}")
    print(f"Best model saved to : {best_path}")

    # Close environments after training.
    train_env.close()
    eval_env.close()


if __name__ == "__main__":
    main()

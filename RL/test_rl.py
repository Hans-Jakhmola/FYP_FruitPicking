import os

from stable_baselines3 import SAC

from ur5_pickplace import UR5PickPlaceEnv


# Path to the trained SAC model to test.
MODEL_PATH = "models/curriculum_stage_2_10cm_PickplaceD_RandS2/best_model/best_model.zip"

# Number of evaluation episodes to run.
NUM_EPISODES = 10


def main():
    # Stop early if the model file cannot be found.
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model not found: {MODEL_PATH}")

    # Create the PyBullet test environment with GUI enabled.
    # spawn_range=0.15 tests random cube positions within the trained range.
    env = UR5PickPlaceEnv(gui=True, spawn_range=0.15)

    # Load the trained SAC policy.
    model = SAC.load(MODEL_PATH)

    try:
        # Run multiple episodes so performance is not judged from one attempt.
        for episode in range(1, NUM_EPISODES + 1):

            # Reset the robot, cube position, reward variables and simulator state.
            # obs is the first observation given to the SAC policy.
            obs, info = env.reset()

            # Track whether the episode has finished.
            done = False

            # Store the total reward collected during this episode.
            episode_reward = 0.0

            # Count how many control steps the policy used.
            step_count = 0

            # Keep stepping until the task succeeds or the environment times out.
            while not done:

                # Predict the next action from the trained SAC policy.
                # deterministic=True removes exploration noise during testing.
                action, _states = model.predict(obs, deterministic=True)

                # Apply the action to the environment.
                # The environment returns the next observation, reward and end flags.
                obs, reward, terminated, truncated, info = env.step(action)

                # Add this step's reward to the episode total.
                episode_reward += reward

                # Count one completed environment step.
                step_count += 1

                # terminated means task success.
                # truncated means the maximum episode length was reached.
                done = terminated or truncated

            # Print a compact summary of the episode result.
            print(
                f"Episode {episode}: "
                f"reward={episode_reward:.3f}, "
                f"steps={step_count}, "
                f"success={terminated}"
            )

            # Print final diagnostic values such as grasping, lift and drop distance.
            print("Final info:", info)

    finally:
        # Always close the simulator, even if an error occurs.
        env.close()


if __name__ == "__main__":
    main()

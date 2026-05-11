# UR5 Pick and Place Environment.
# The SAC policy controls small end-effector movements and the gripper.
# The task is to align with the cube, grasp it, lift it, and move it to DROP_POS.

import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
from pybullet_planning import joint_from_name, link_from_name

from Environment import environment as BaseEnvironment


# Joint names are converted to PyBullet joint IDs after the robot is loaded.
ARM_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

GRIPPER_JOINT_NAMES = [
    "finger_joint", "left_inner_finger_joint", "left_inner_knuckle_joint",
    "right_outer_knuckle_joint", "right_inner_finger_joint", "right_inner_knuckle_joint",
]

# Main task geometry.
PRE_GRASP_OFFSET = 0.03
DISTANCE_THRESHOLD = 0.1
DROP_POS = np.array([0.3, 0.3, 0.2], dtype=np.float32)

# Reward timing and shaping constants. Manually tuned.
ALIGN_PATIENCE = 25

GRASP_BONUS = 20.0
GRASP_HOLD_MAX = 3.0
GRASP_DECAY_STEPS = 20
MIN_LIFT_BEFORE_TRANSPORT = 0.02
LIFT_PROGRESS_SCALE = 120.0
LIFT_HEIGHT_SCALE = 20.0
LIFTED_BONUS = 25.0
LOW_DRAG_PENALTY_SCALE = 5.0
LIFT_DECAY_STEPS = 20

TRANSPORT_PROGRESS_SCALE = 100.0
DROP_DISTANCE_SCALE = 15.0
GRASP_PERSISTENCE = 0.0
STILLNESS_PENALTY = -0.5
STILLNESS_THRESHOLD = 0.001

# Anti-toggle values stop the policy from farming repeated open/close behaviour.
UNGRIP_PENALTY = -25.0
GRIPPER_HYSTERESIS = 0.5

SUCCESS_BONUS = 100.0


class UR5PickPlaceEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self, gui: bool = False, max_steps: int = 200, spawn_range: float = 0.0):
        super().__init__()

        self.gui = gui
        self.max_steps = max_steps
        self.spawn_range = spawn_range
        self.sim_steps_per_action = 24
        self.control_dt = 1.0 / 240.0

        self.sim = None
        self.robot_id = None
        self.fruit_id = None

        self.arm_joint_ids = []
        self.gripper_joint_ids = []
        self.ee_link_id = None
        self.left_pad_link_id = None
        self.right_pad_link_id = None

        # Episode state variables are reset at the start of every episode.
        self.step_count = 0
        self.align_steps = 0
        self.grasp_steps = 0
        self.was_grasping = False
        self.one_shot_grasp_awarded = False
        self.had_grasp_this_episode = False
        self.gripper_locked = False
        self.lift_bonus_awarded = False

        # Previous values are used to calculate progress-based reward terms.
        self.prev_lift_amount = 0.0
        self.prev_dist_to_drop = None

        self.gripper_state = -1.0
        self.fixed_ee_orn = None
        self.rest_poses = None

        self.initial_fruit_pos = None
        self.pre_grasp_pos = None

        # Workspace limits stop the policy from commanding unreachable or unsafe Cartesian targets.
        self.workspace_low = np.array([0.20, -0.40, 0.05], dtype=np.float32)
        self.workspace_high = np.array([0.85, 0.40, 0.85], dtype=np.float32)
        self.ee_delta_scale = np.array([0.01, 0.01, 0.01], dtype=np.float32)

        # Action = small Cartesian delta in x/y/z plus one gripper command.
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4,),
            dtype=np.float32,
        )

        # Observation = gripper position, fruit position, relative fruit vector,
        # gripper state, and fruit-to-drop vector.
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(13,),
            dtype=np.float32,
        )

    def _setup_robot_indices(self):
        # Find the PyBullet joint and link IDs needed for control and contact checks.
        self.arm_joint_ids = [joint_from_name(self.robot_id, n) for n in ARM_JOINT_NAMES]
        self.gripper_joint_ids = [joint_from_name(self.robot_id, n) for n in GRIPPER_JOINT_NAMES]
        self.ee_link_id = link_from_name(self.robot_id, "ee_link")
        self.left_pad_link_id = link_from_name(self.robot_id, "left_inner_finger_pad")
        self.right_pad_link_id = link_from_name(self.robot_id, "right_inner_finger_pad")

    def _get_ee_pos(self):
        # Return the current end-effector position.
        return np.array(p.getLinkState(self.robot_id, self.ee_link_id)[4], dtype=np.float32)

    def _get_ee_orn(self):
        # Return the current end-effector orientation.
        return p.getLinkState(self.robot_id, self.ee_link_id)[5]

    def _get_pad_positions(self):
        # Return the world positions of the two inner gripper pads.
        l = np.array(p.getLinkState(self.robot_id, self.left_pad_link_id)[4], dtype=np.float32)
        r = np.array(p.getLinkState(self.robot_id, self.right_pad_link_id)[4], dtype=np.float32)
        return l, r

    def _get_gripper_centre(self):
        # Approximate the grasp centre as the midpoint between both finger pads.
        l, r = self._get_pad_positions()
        return 0.5 * (l + r)

    def _get_fruit_pos(self):
        # Return the current cube/fruit position.
        pos, _ = p.getBasePositionAndOrientation(self.fruit_id)
        return np.array(pos, dtype=np.float32)

    def _get_obs(self):
        # Build the 13D observation vector used by the SAC policy.
        gc = self._get_gripper_centre()
        fruit = self._get_fruit_pos()
        rel = fruit - gc
        rel_to_drop = DROP_POS - fruit

        return np.concatenate(
            [gc, fruit, rel, [self.gripper_state], rel_to_drop]
        ).astype(np.float32)

    def _apply_ee_delta(self, dx, dy, dz):
        # Apply the policy's Cartesian movement command through inverse kinematics.
        ee = self._get_ee_pos()
        target = np.clip(
            ee + self.ee_delta_scale * np.array([dx, dy, dz], dtype=np.float32),
            self.workspace_low,
            self.workspace_high,
        )

        # IK converts the desired end-effector target into UR5 joint targets.
        ik = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link_id,
            targetPosition=target.tolist(),
            targetOrientation=self.fixed_ee_orn,
            restPoses=self.rest_poses,
        )

        for joint_id, q in zip(self.arm_joint_ids, ik[:len(self.arm_joint_ids)]):
            p.setJointMotorControl2(
                self.robot_id,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=float(q),
                force=200.0,
            )

    def _apply_gripper(self, grip_action):
        # Apply the gripper command with hysteresis and post-grasp locking.
        grip = float(np.clip(grip_action, -1.0, 1.0))

        # Once a valid grasp is detected, the gripper is forced to remain closed.
        if self.gripper_locked:
            self.gripper_state = 1.0
        else:
            if grip > GRIPPER_HYSTERESIS:
                self.gripper_state = 1.0
            elif grip < -GRIPPER_HYSTERESIS:
                self.gripper_state = -1.0

        target = 0.8 if self.gripper_state > 0.0 else 0.0

        for joint_id in self.gripper_joint_ids:
            p.setJointMotorControl2(
                self.robot_id,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=target,
                force=10.0,
            )

    def _is_grasping(self):
        # A grasp is valid only when both inner finger pads contact the fruit.
        left = p.getContactPoints(
            bodyA=self.robot_id,
            bodyB=self.fruit_id,
            linkIndexA=self.left_pad_link_id,
        )
        right = p.getContactPoints(
            bodyA=self.robot_id,
            bodyB=self.fruit_id,
            linkIndexA=self.right_pad_link_id,
        )
        return len(left) > 0 and len(right) > 0

    def _is_success(self):
        # Success requires the fruit to be grasped and close to the drop position.
        fruit = self._get_fruit_pos()
        dist_to_drop = float(np.linalg.norm(fruit - DROP_POS))
        return self._is_grasping() and dist_to_drop < DISTANCE_THRESHOLD

    def _compute_reward(self):
        # Compute reward terms for reaching, grasping, lifting and transport.
        gc = self._get_gripper_centre()
        fruit = self._get_fruit_pos()
        grasping = self._is_grasping()

        raw_lift_amount = float(fruit[2] - self.initial_fruit_pos[2])
        lift_amount = max(0.0, raw_lift_amount)
        lifted = lift_amount >= MIN_LIFT_BEFORE_TRANSPORT

        dist_to_pregrasp = float(np.linalg.norm(gc - self.pre_grasp_pos))
        dist_to_fruit = float(np.linalg.norm(gc - fruit))
        dist_to_drop = float(np.linalg.norm(fruit - DROP_POS))
        near_pregrasp = dist_to_pregrasp < DISTANCE_THRESHOLD

        # Progress rewards compare the current state against the previous step.
        lift_progress = max(0.0, lift_amount - self.prev_lift_amount)

        if self.prev_dist_to_drop is None:
            transport_progress = 0.0
        else:
            transport_progress = self.prev_dist_to_drop - dist_to_drop

        reward = 0.0
        info_grasp_bonus = 0.0
        info_hold_bonus = 0.0
        info_lift_reward = 0.0
        info_lifted_bonus = 0.0
        info_drop_reward = 0.0
        info_transport_progress = 0.0
        info_low_drag_penalty = 0.0
        info_stillness_penalty = 0.0
        info_ungrip_pen = 0.0

        if not grasping:
            # Before grasping, the policy is rewarded for moving toward the pre-grasp point.
            diff = gc - self.pre_grasp_pos

            reward = -(
                5.0 * abs(float(diff[0]))
                + 5.0 * abs(float(diff[1]))
                + 1.0 * abs(float(diff[2]))
            )

            if near_pregrasp:
                self.align_steps += 1
            else:
                self.align_steps = 0

            # After waiting above the fruit, approach reward is reduced to encourage closing.
            if self.align_steps >= ALIGN_PATIENCE:
                reward = -0.1

            if self.was_grasping and self.had_grasp_this_episode:
                reward += UNGRIP_PENALTY
                info_ungrip_pen = UNGRIP_PENALTY

            self.grasp_steps = 0
            self.was_grasping = False

        else:
            # Once grasped, keep the gripper locked and switch to lift/transport reward.
            self.gripper_locked = True
            self.grasp_steps += 1
            self.had_grasp_this_episode = True
            self.was_grasping = True

            if not self.one_shot_grasp_awarded:
                info_grasp_bonus = GRASP_BONUS
                self.one_shot_grasp_awarded = True

            decay = float(np.exp(-self.grasp_steps / GRASP_DECAY_STEPS * 3.0))
            info_hold_bonus = GRASP_HOLD_MAX * decay

            # Lift reward is strongest early, then decays so transport becomes more important.
            lift_decay = float(np.exp(-self.grasp_steps / LIFT_DECAY_STEPS * 3.0))
            info_lift_reward = (
                LIFT_PROGRESS_SCALE * lift_progress
                + LIFT_HEIGHT_SCALE * lift_amount
            ) * lift_decay

            # Transport reward is positive when the fruit gets closer to DROP_POS.
            info_transport_progress = TRANSPORT_PROGRESS_SCALE * transport_progress
            info_drop_reward = (
                info_transport_progress
                - DROP_DISTANCE_SCALE * dist_to_drop
            )

            fruit_speed = abs(transport_progress) + abs(lift_progress)
            if fruit_speed < STILLNESS_THRESHOLD:
                info_stillness_penalty = STILLNESS_PENALTY

            reward = (
                info_grasp_bonus
                + info_hold_bonus
                + info_lift_reward
                + info_drop_reward
                + info_stillness_penalty
            )

        success = self._is_success()

        if success:
            reward += SUCCESS_BONUS

        # Store current values for progress calculations on the next step.
        self.prev_lift_amount = lift_amount
        self.prev_dist_to_drop = dist_to_drop

        info = {
            "is_grasping": float(grasping),
            "is_success": float(success),
            "gripper_locked": float(self.gripper_locked),
            "dist_to_pregrasp": dist_to_pregrasp,
            "dist_to_fruit": dist_to_fruit,
            "dist_to_drop": dist_to_drop,
            "fruit_z": float(fruit[2]),
            "lift_amount": lift_amount,
            "lift_progress": lift_progress,
            "lifted": float(lifted),
            "align_steps": float(self.align_steps),
            "grasp_steps": float(self.grasp_steps),
            "hold_bonus": info_hold_bonus,
            "grasp_bonus": info_grasp_bonus,
            "lift_reward": info_lift_reward,
            "lifted_bonus": info_lifted_bonus,
            "drop_reward": info_drop_reward,
            "transport_progress": info_transport_progress,
            "low_drag_penalty": info_low_drag_penalty,
            "stillness_penalty": info_stillness_penalty,
            "ungrip_penalty": info_ungrip_pen,
            "in_transport_phase": float(lifted),
        }

        return float(reward), info

    def reset(self, seed=None, options=None):
        # Reset the simulator, fruit position and all episode state variables.
        super().reset(seed=seed)

        if self.sim is not None:
            self.sim.close()
            self.sim = None

        self.sim = BaseEnvironment()

        randomize = self.spawn_range > 0.0
        self.sim._x_range = self.spawn_range
        self.sim._y_range = self.spawn_range
        self.sim.initialise(self.gui, randomize_fruit=randomize)

        # Every GUI reset restores the same camera angle for testing.
        if self.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.25,
                cameraYaw=45,
                cameraPitch=-35,
                cameraTargetPosition=[0.45, 0.0, 0.25],
            )

        self.robot_id = self.sim.ur5
        self.fruit_id = self.sim.fruit
        self._setup_robot_indices()

        # Let the robot and cube settle before the first observation.
        for _ in range(50):
            p.stepSimulation()
            if self.gui:
                time.sleep(self.control_dt)

        self.fixed_ee_orn = self._get_ee_orn()
        self.rest_poses = [p.getJointState(self.robot_id, j)[0] for j in self.arm_joint_ids]

        self.step_count = 0
        self.align_steps = 0
        self.grasp_steps = 0
        self.was_grasping = False
        self.one_shot_grasp_awarded = False
        self.had_grasp_this_episode = False
        self.gripper_locked = False
        self.lift_bonus_awarded = False
        self.gripper_state = -1.0

        self.initial_fruit_pos = self._get_fruit_pos().copy()
        self.pre_grasp_pos = self.initial_fruit_pos + np.array(
            [0.0, 0.0, PRE_GRASP_OFFSET],
            dtype=np.float32,
        )

        self.prev_lift_amount = 0.0
        self.prev_dist_to_drop = float(np.linalg.norm(self.initial_fruit_pos - DROP_POS))

        return self._get_obs(), {}

    def step(self, action):
        # Apply one SAC action, advance physics, then return Gymnasium step outputs.
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)

        dx, dy, dz, grip = action

        self._apply_ee_delta(float(dx), float(dy), float(dz))
        self._apply_gripper(float(grip))

        for _ in range(self.sim_steps_per_action):
            p.stepSimulation()
            if self.gui:
                time.sleep(self.control_dt)

        self.step_count += 1

        obs = self._get_obs()
        reward, info = self._compute_reward()
        terminated = self._is_success()
        truncated = self.step_count >= self.max_steps

        return obs, reward, terminated, truncated, info

    def close(self):
        # Close the PyBullet simulation connection.
        if self.sim is not None:
            self.sim.close()
            self.sim = None

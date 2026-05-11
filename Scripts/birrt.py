import csv
import os
import time
import numpy as np
import pybullet as p
import pybullet_planning as pp

from pybullet_planning import (
    draw_aabb,
    set_joint_positions,
    create_attachment,
    get_collision_fn,
    get_disabled_collisions,
    BASE_LINK,
    joint_from_name,
    link_from_name,
)

from planner_2d_utils import create_aabb_box
from Environment import environment

np.random.seed(0)
_render_lock_depth = 0


# Temporarily disables rendering while planning so the GUI does not show sudden robot teleports.
# A reference counter is used because planning functions can call other planning functions.
class RenderLock:
    # Enter the render lock. Rendering is disabled only for the first active lock.
    def __enter__(
        self,
    ):
        #Disable PyBullet rendering while a planning block is active.
        global _render_lock_depth
        if _render_lock_depth == 0:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        _render_lock_depth += 1
        return self

    # Exit the render lock. Rendering is restored only when all nested locks have finished.
    def __exit__(
        self,
        *args,
    ):
        #Restore rendering only when the outermost planning block finishes.
        global _render_lock_depth
        _render_lock_depth -= 1
        if _render_lock_depth == 0:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)


# initialise environment
env = environment()
env.initialise(gui=True)


# A simple box around the trunk is used for collision checking during planning.
# This is much faster than checking against the full mesh.
TRUNK_CENTER = np.array([0.63, -0.054, 0.28])
TRUNK_SIZE = np.array([0.20, 0.22, 1.3], dtype=float)

trunk_col_shape = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=(TRUNK_SIZE / 2.0).tolist(),
)
trunk_vis_shape = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=(TRUNK_SIZE / 2.0).tolist(),
    rgbaColor=[1.0, 0.5, 0.0, 0.12],
)
tree_collision_body = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=trunk_col_shape,
    baseVisualShapeIndex=trunk_vis_shape,
    basePosition=TRUNK_CENTER.tolist(),
)
p.setCollisionFilterPair(tree_collision_body, env.tree, -1, -1, 0) #stops potential collision between collision box and meshes
p.setCollisionFilterPair(tree_collision_body, env.fruit, -1, -1, 0)

# Draw debug AABBs so the collision volumes are visible in the GUI.
SCALE = 0.13
raw_x_centre = (-1.090 + 0.210) / 2.0
raw_y_centre = (0.000 + 7.000) / 2.0
raw_z_centre = (-1.197 + 0.403) / 2.0
tree_base = np.array([0.7, -0.1, 0.0])
aabb_centre = np.array([
    tree_base[0] + raw_x_centre * SCALE - 0.03,
    tree_base[1] + (-raw_z_centre * SCALE),
    tree_base[2] + raw_y_centre * SCALE,
])
TREE_VIS_SIZE = np.array([0.200, 0.193, 0.864], dtype=float)
tree_obstacle_aabb = create_aabb_box(center=aabb_centre, extents=tuple(TREE_VIS_SIZE))
draw_aabb(tree_obstacle_aabb)
draw_aabb(create_aabb_box(center=TRUNK_CENTER, extents=tuple(TRUNK_SIZE)))

# Robot constants: joint limits, weights and all links and joints used for planning and collision checking.
JOINT_LOWER = [-3.14159] * 6
JOINT_UPPER = [3.14159] * 6
JOINT_RANGE = [u - l for l, u in zip(JOINT_LOWER, JOINT_UPPER)]
JOINT_WEIGHTS = np.array([6, 5, 4, 3, 2, 1], dtype=float)
HOME_JOINTS = np.array(env.ur5_start_conf[:6])

ARM_JOINTS = [joint_from_name(env.ur5, n) for n in [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]]

GRIPPER_JOINTS = [joint_from_name(env.ur5, n) for n in [
    "finger_joint", "left_inner_finger_joint", "left_inner_knuckle_joint",
    "right_outer_knuckle_joint", "right_inner_finger_joint", "right_inner_knuckle_joint",
]]

TOOL_LINK = link_from_name(env.ur5, "robotiq_arg2f_base_link")

GRIPPER_CONTACT_LINKS = set(link_from_name(env.ur5, n) for n in [
    "ee_link", "robotiq_arg2f_base_link",
    "left_outer_finger", "left_inner_finger", "left_inner_finger_pad",
    "right_outer_finger", "right_inner_finger", "right_inner_finger_pad",
])

ALL_GRIPPER_LINKS = set(link_from_name(env.ur5, n) for n in [
    "wrist_3_link", "ee_link", "robotiq_arg2f_base_link",
    "left_outer_knuckle", "left_outer_finger", "left_inner_finger",
    "left_inner_finger_pad", "left_inner_knuckle", "right_outer_knuckle",
    "right_outer_finger", "right_inner_finger", "right_inner_finger_pad",
    "right_inner_knuckle",
])

LIMITS = {
    ARM_JOINTS[0]: (JOINT_LOWER[0], JOINT_UPPER[0]),
    ARM_JOINTS[1]: (JOINT_LOWER[1], JOINT_UPPER[1]),
    ARM_JOINTS[2]: (JOINT_LOWER[2], JOINT_UPPER[2]),
    ARM_JOINTS[3]: (JOINT_LOWER[3], JOINT_UPPER[3]),
    ARM_JOINTS[4]: (JOINT_LOWER[4], JOINT_UPPER[4]),
    ARM_JOINTS[5]: (JOINT_LOWER[5], JOINT_UPPER[5]),
}

# Weighted joint-space distance function used by BiRRT for nearest-neighbour and path-cost calculations.
DISTANCE_FN = pp.get_distance_fn(env.ur5, ARM_JOINTS, weights=JOINT_WEIGHTS)

# Random joint-configuration sampler used by BiRRT, limited to the defined UR5 joint bounds.
SAMPLE_FN = pp.get_sample_fn(env.ur5, ARM_JOINTS, custom_limits=LIMITS)

# Joint-space interpolation function used by BiRRT and path smoothing.
EXTEND_FN = pp.get_extend_fn(env.ur5, ARM_JOINTS, resolutions=[0.08] * len(ARM_JOINTS))

# Finer joint-space interpolation used only for checking direct start-to-goal paths.
DIRECT_EXTEND_FN = pp.get_extend_fn(env.ur5, ARM_JOINTS, resolutions=[0.05] * len(ARM_JOINTS))

# Grasp / carry information, goes over gripper centre fruit spawning and approach direction constants
FRUIT_RADIUS = 0.135
FRUIT_Z = 0.520
GRIP_CENTER_LOCAL = np.array([0.0, 0.0, 0.125])
GRASP_CLEARANCE = 0.015
POSTDROP_RETREAT = 0.08

DROP_POS = np.array([0.3, 0.3, 0.2]) #drop position for fruit after carrying

# position and approach direction. Here, ORIENTATION is used as a standard wrist pose candidate when planning to the pre-drop area and as the maintained tool
# orientation during short drop/retreat Cartesian motions. makes orientation 90 degrees pitch no roll or yaw
ORIENTATION = p.getQuaternionFromEuler([0, np.pi / 2, 0])


# Height used by the fallback overhead carry waypoints.
# If direct planning to the drop area fails, the planner tries routing the
# carried fruit through waypoints at this z-level to improve clearance over the tree/trunk collision body.
SAFE_CARRY_Z = 0.72
OVERHEAD_WAYPOINTS = [
    np.array([0.35, 0.10, SAFE_CARRY_Z]),
    np.array([0.30, 0.25, SAFE_CARRY_Z]),
    np.array([0.25, 0.35, SAFE_CARRY_Z]),
]

# disable collision between the robot links to avoid self collision errors during planning.
SELF_COLLISION_DISABLED_LINK_NAMES = [
    ("base_link", "shoulder_link"),
    ("ee_link", "wrist_1_link"),
    ("ee_link", "wrist_2_link"),
    ("ee_link", "wrist_3_link"),
    ("forearm_link", "upper_arm_link"),
    ("forearm_link", "wrist_1_link"),
    ("shoulder_link", "upper_arm_link"),
    ("wrist_1_link", "wrist_2_link"),
    ("wrist_1_link", "wrist_3_link"),
    ("wrist_2_link", "wrist_3_link"),
    ("ee_link", "robotiq_arg2f_base_link"),
    ("robotiq_arg2f_base_link", "left_inner_knuckle"),
    ("robotiq_arg2f_base_link", "right_inner_knuckle"),
    ("robotiq_arg2f_base_link", "left_outer_knuckle"),
    ("robotiq_arg2f_base_link", "right_outer_knuckle"),
]

# global values for fruit
FRUIT_POS = np.array([TRUNK_CENTER[0] - FRUIT_RADIUS, TRUNK_CENTER[1], FRUIT_Z])
FRUIT_ORN = p.getBasePositionAndOrientation(env.fruit)[1]
fruit_constraint = None
grip_constraint = None



# Returns the largest single-joint change between two configurations.
# Used to reject IK solutions that jump too far and cause wrist or arm flips.
def max_joint_delta(
    q1,
    q2,
):
    #Return the largest absolute movement made by any single joint.
    q1 = np.array(q1, dtype=float)
    q2 = np.array(q2, dtype=float)
    joint_deltas = np.abs(q2 - q1)
    return float(np.max(joint_deltas))


# Calculates the total cost of a joint-space path.
# It uses pybullet_planning's path cost when available, otherwise sums weighted joint distances.
def path_cost(
    path,
):
    #Calculate the total weighted cost of a planned joint path.
    if path is None or len(path) < 2:
        return np.inf
    else:

        return float(pp.compute_path_cost(path, DISTANCE_FN))

# Removes duplicate IK seed configurations.
# This avoids wasting time trying the same or nearly identical seed repeatedly.
def dedupe_confs(
    conf_list,
    tol=1e-4,
):
    #Remove repeated or nearly repeated joint configurations.
    unique = []

    for conf in conf_list:
        if conf is None:
            continue

        conf = [float(x) for x in conf]
        already_seen = any(
            np.linalg.norm(np.array(conf) - np.array(u)) < tol
            for u in unique
        )
        if not already_seen:
            unique.append(conf)

    return unique


# Enables or disables collision checks between every robot link and the fruit.
# This is needed after attachment so the held fruit does not collide with the gripper carrying it.
def disable_robot_fruit_collisions(
    disable=True,
):
    #Toggle collision checks between the robot and the carried fruit.
    flag = 0 if disable else 1

    # Loop over base link plus all robot joints so the setting applies to the whole robot.
    for link_idx in range(-1, p.getNumJoints(env.ur5)):
        p.setCollisionFilterPair(env.ur5, env.fruit, link_idx, -1, flag)


# Checks whether the fruit is touching one of the gripper contact links.
# This confirms that the close-gripper action has reached the fruit before creating the fixed grasp constraint.
def get_gripper_contact(
):
    #Return the first contact point between the fruit and any gripper link.
    contacts = p.getContactPoints(bodyA=env.ur5, bodyB=env.fruit)

    # PyBullet contact index 3 stores the robot link involved in contact.
    for contact in contacts:
        robot_link_index = contact[3]
        if robot_link_index in GRIPPER_CONTACT_LINKS:
            return contact

    return None


# Builds the collision checker used by BiRRT and IK filtering.
# It wraps pybullet_planning.get_collision_fn while adding project-specific tree, self-collision and fruit-carrying exclusions.
def build_collision_fn(
    attachments=None,
    extra_obstacles=None,
    carrying=False,
    include_tree=True,
):
    #Build the project-specific collision checker used by IK and BiRRT.
    attachments = attachments or [] #if none just create empty list
    extra_obstacles = extra_obstacles or [] #To do/FUTURE WORK - add support for extra obstacles in the environment and include them in this function

    disabled_collisions = get_disabled_collisions(env.ur5, SELF_COLLISION_DISABLED_LINK_NAMES)
    extra_disabled_collisions = set()

    if carrying: #if carrying fruit disable collisions between gripper and fruit
        for link_idx in ALL_GRIPPER_LINKS:
            extra_disabled_collisions.add(((env.ur5, link_idx), (env.fruit, BASE_LINK)))

    obstacles = ([tree_collision_body] if include_tree else []) + list(extra_obstacles)

    return get_collision_fn(
        env.ur5,
        ARM_JOINTS,
        obstacles=obstacles,
        attachments=attachments,
        self_collisions=True,
        disabled_collisions=disabled_collisions,
        extra_disabled_collisions=extra_disabled_collisions,
        custom_limits={},
    )

# Converts a vector to unit length.
# Used for grasp orientation code so approach directions are consistent.
def normalise(
    v,
):
    #Return a unit-length version of a vector, preserving near-zero vectors.
    v = np.array(v, dtype=float)
    n = np.linalg.norm(v)

    if n > 1e-8:
        return v / n
    return v


# Converts a 3x3 rotation matrix into a PyBullet quaternion.
# This is used after constructing a tool frame from approach-direction vectors.
def rotation_matrix_to_quaternion(
    R,
):
    #Convert a rotation matrix into a quaternion
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return [qx, qy, qz, qw]


# Rotates a local-frame vector into the world frame using a quaternion.
# Used to compute where the gripper pinch point lies relative to the fruit.
def local_to_world_vector(
    q,
    v_local,
):
    #Rotate a vector from the tool's local frame into the world frame
    rotation_matrix = np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3)
    local_vector = np.array(v_local, dtype=float)
    return rotation_matrix @ local_vector


# Creates a gripper orientation from an approach direction and roll angle.
# The approach direction becomes the tool z-axis, then roll rotates the gripper around that axis.
def quaternion_from_approach_direction(
    approach_dir,
    roll=0.0,
):
    #Build a gripper quaternion from a desired approach direction and roll.
    z_axis = normalise(approach_dir)
    world_up = (
        np.array([0.0, 0.0, 1.0])
        if abs(np.dot(z_axis, [0, 0, 1])) < 0.95
        else np.array([1.0, 0.0, 0.0])
    )
    x_axis = normalise(np.cross(world_up, z_axis))
    y_axis = normalise(np.cross(z_axis, x_axis))
    base_q = rotation_matrix_to_quaternion(np.column_stack((x_axis, y_axis, z_axis)))
    roll_q = p.getQuaternionFromEuler([0.0, 0.0, roll])
    return p.multiplyTransforms([0, 0, 0], base_q, [0, 0, 0], roll_q)[1]


# Computes the tool pose needed to place the gripper pinch point at the fruit.
# Clearance controls whether the pose is a pre-grasp standoff or the final grasp pose.
def compute_tool_pose_from_fruit(
    fruit_pos,
    approach_dir,
    roll,
    clearance,
):
    #Compute the tool pose whose pinch point reaches the fruit with clearance.
    tool_orn = quaternion_from_approach_direction(approach_dir, roll)
    backoff_local = np.array([
        GRIP_CENTER_LOCAL[0],
        GRIP_CENTER_LOCAL[1],
        GRIP_CENTER_LOCAL[2] + clearance,
    ])
    tool_pos = np.array(fruit_pos, dtype=float) - local_to_world_vector(tool_orn, backoff_local)
    return tool_pos, tool_orn


# Removes near-duplicate approach directions.
# This keeps the grasp search compact while preserving distinct candidate directions.
def unique_directions(
    direction_list,
    cos_tol=0.985,
):
    #Filter out approach directions that point almost the same way.
    unique = []
    for d in direction_list:
        d = normalise(d)
        if np.linalg.norm(d) < 1e-8:
            continue
        if not any(np.dot(d, u) > cos_tol for u in unique):
            unique.append(d)
    return unique


# Classifies the fruit position relative to the trunk as front, left or right
# The case selects different approach and roll candidates for grasp search.
def get_fruit_case(
    fruit_pos,
    trunk_center,
):
    #Classify fruit location relative to the trunk as front, left or right
    delta = np.array(fruit_pos, dtype=float) - np.array(trunk_center, dtype=float)
    dx, dy = float(delta[0]), float(delta[1])
    if abs(dx) >= abs(dy):
        return "front"
    return "left" if dy > 0.0 else "right"


# Generates candidate approach directions for the fruit's side of the tree.
# It starts with the inward direction and adds angled variants to improve IK coverage.
def get_case_aware_approach_dirs(
    fruit_pos,
    trunk_center,
):
    #Generate approach directions based on where the fruit is on the tree.
    fruit_case = get_fruit_case(fruit_pos, trunk_center)
    fruit_to_trunk_xy = (np.array(fruit_pos) - np.array(trunk_center))[:2]
    outward = normalise(
        np.array([*fruit_to_trunk_xy, 0.0])
        if np.linalg.norm(fruit_to_trunk_xy) > 1e-8
        else np.array([-1.0, 0.0, 0.0])
    )
    inward = -outward
    tangent = normalise(np.array([-outward[1], outward[0], 0.0]))
    up = np.array([0.0, 0.0, 1.0])

    candidates = [
        inward,
        normalise(inward + 0.15 * up),
        normalise(inward + 0.30 * up),
    ]
    if fruit_case in ("left", "right"):
        candidates += [
            normalise(inward + 0.30 * tangent),
            normalise(inward - 0.30 * tangent),
            normalise(inward + 0.22 * tangent + 0.18 * up),
            normalise(inward - 0.22 * tangent + 0.18 * up),
        ]
    return unique_directions(candidates)


# Returns gripper roll angles to try for a given fruit side.
# More roll options are used for side fruit because those poses are harder to reach.
def get_case_aware_rolls(
    fruit_case,
):
    #Return candidate gripper roll angles for the selected fruit case.
    if fruit_case == "front":
        return [0.0, np.pi / 4, -np.pi / 4, np.pi / 2, -np.pi / 2]

    return [0.0, np.pi / 4, -np.pi / 4, np.pi / 2, -np.pi / 2, np.pi]



# Attaches the fruit to the tree with a fixed constraint.
# This simulates fruit growing on the tree until the gripper makes contact and detaches it.
def attach_fruit_to_tree(
):
    #Place the fruit at FRUIT_POS and attach it rigidly to the tree.
    global fruit_constraint
    if fruit_constraint is not None:
        p.removeConstraint(fruit_constraint)
        fruit_constraint = None

    tree_pos, tree_orn = p.getBasePositionAndOrientation(env.tree)
    p.resetBasePositionAndOrientation(env.fruit, FRUIT_POS.tolist(), FRUIT_ORN)
    p.resetBaseVelocity(env.fruit, [0, 0, 0], [0, 0, 0])
    p.setCollisionFilterPair(env.tree, env.fruit, -1, -1, 0)

    inv_pos, inv_orn = p.invertTransform(tree_pos, tree_orn)
    parent_local_pos, parent_local_orn = p.multiplyTransforms(
        inv_pos,
        inv_orn,
        FRUIT_POS.tolist(),
        FRUIT_ORN,
    )
    fruit_constraint = p.createConstraint(
        parentBodyUniqueId=env.tree,
        parentLinkIndex=-1,
        childBodyUniqueId=env.fruit,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=parent_local_pos,
        parentFrameOrientation=parent_local_orn,
        childFramePosition=[0, 0, 0],
        childFrameOrientation=[0, 0, 0, 1],
    )
    p.changeConstraint(fruit_constraint, maxForce=50000)
    for _ in range(10):
        env.step()


# Removes the tree-to-fruit constraint and restores fruit-tree collision.
# Called once the gripper has made contact so the fruit can be carried away.
def detach_fruit_from_tree(
):
    #Remove the rigid tree-fruit attachment so the fruit can be carried.
    global fruit_constraint
    if fruit_constraint is not None:
        p.removeConstraint(fruit_constraint)
        fruit_constraint = None
    p.setCollisionFilterPair(env.tree, env.fruit, -1, -1, 1)


# Attaches the fruit to the gripper with a fixed constraint.
# This simplified grasp model keeps the fruit at the gripper pinch point during transport.
def attach_fruit_to_gripper(
):
    #Attach the fruit rigidly to the gripper pinch point after contact.
    global grip_constraint
    if grip_constraint is not None:
        p.removeConstraint(grip_constraint)
        grip_constraint = None

    tool_pos, tool_orn = pp.get_link_pose(env.ur5, TOOL_LINK)
    tool_pos = np.array(tool_pos, dtype=float)
    desired_fruit_pos = tool_pos + local_to_world_vector(tool_orn, GRIP_CENTER_LOCAL)
    p.resetBasePositionAndOrientation(env.fruit, desired_fruit_pos.tolist(), tool_orn)
    p.resetBaseVelocity(env.fruit, [0, 0, 0], [0, 0, 0])

    grip_constraint = p.createConstraint(
        parentBodyUniqueId=env.ur5,
        parentLinkIndex=TOOL_LINK,
        childBodyUniqueId=env.fruit,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=GRIP_CENTER_LOCAL.tolist(),
        parentFrameOrientation=[0, 0, 0, 1],
        childFramePosition=[0, 0, 0],
        childFrameOrientation=[0, 0, 0, 1],
    )
    p.changeConstraint(grip_constraint, maxForce=500)
    disable_robot_fruit_collisions(disable=True)


# Removes the gripper-to-fruit constraint after release.
# Robot-fruit collisions are re-enabled once the fruit is no longer being carried.
def detach_fruit_from_gripper(
):
    #Remove the gripper-fruit attachment when the fruit is released.
    global grip_constraint
    if grip_constraint is not None:
        p.removeConstraint(grip_constraint)
        grip_constraint = None
    disable_robot_fruit_collisions(disable=False)


# Solves IK for one target pose using a list of seed configurations.
# Each IK result is checked for target-position error, joint jump size and collision before being accepted.
def solve_collision_free_ik_seeded(
    target_pos,
    target_orn,
    collision_fn,
    seed_confs,
    attempts_random=0,
    pos_tol=0.03,
    reference_conf=None,
    max_reference_delta=None,
):
    
    #Solve IK for one target pose using seeded rest poses.

    #The function tries each seed, checks that the tool reaches the target,
    #rejects excessive joint jumps, and finally rejects colliding solutions.
    saved_conf = pp.get_joint_positions(env.ur5, ARM_JOINTS)
    seeds = dedupe_confs(
        ([list(reference_conf)] if reference_conf is not None else [])
        + [list(s) for s in seed_confs if s is not None]
    )
    while len(seeds) < len(seed_confs) + attempts_random:
        seeds.append(np.random.uniform(JOINT_LOWER, JOINT_UPPER).tolist())

    with RenderLock(): #lock renfering while moving the robot for IK attempts to avoid visible teleports in the GUI.
        try:
                # Try each seed because IK can converge to different arm configurations.
            for seed in seeds:
                if target_orn is not None:
                    ik = p.calculateInverseKinematics(
                        env.ur5,
                        TOOL_LINK,
                        target_pos,
                        target_orn,
                        lowerLimits=JOINT_LOWER,
                        upperLimits=JOINT_UPPER,
                        jointRanges=JOINT_RANGE,
                        restPoses=seed,
                        maxNumIterations=160,
                        residualThreshold=1e-5,
                    )
                else:
                    ik = p.calculateInverseKinematics(
                        env.ur5,
                        TOOL_LINK,
                        target_pos,
                        lowerLimits=JOINT_LOWER,
                        upperLimits=JOINT_UPPER,
                        jointRanges=JOINT_RANGE,
                        restPoses=seed,
                        maxNumIterations=160,
                        residualThreshold=1e-5,
                    )
                conf = [float(np.clip(ik[i], JOINT_LOWER[i], JOINT_UPPER[i])) for i in range(6)]

                if reference_conf is not None and max_reference_delta is not None:
                    if max_joint_delta(reference_conf, conf) > max_reference_delta:
                        continue

                set_joint_positions(env.ur5, ARM_JOINTS, conf)
                actual_pos, _ = pp.get_link_pose(env.ur5, TOOL_LINK)
                pos_err = np.linalg.norm(np.array(actual_pos) - np.array(target_pos))
                if pos_err <= pos_tol and not collision_fn(conf):
                    return conf
        finally:
            set_joint_positions(env.ur5, ARM_JOINTS, saved_conf)
    return None


# Tries multiple target positions and orientations and returns the lowest-cost valid IK solution.
# This is used when several pre-drop poses may work and the planner should choose the easiest one.
def solve_collision_free_ik_multi_pose(
    position_candidates,
    orientation_candidates,
    collision_fn,
    seed_confs,
    attempts_random=0,
    pos_tol=0.03,
    reference_conf=None,
    max_reference_delta=None,
):
    #Try pose candidates and return the valid IK solution with lowest joint cost
    best, best_score = None, np.inf
    ref = reference_conf if reference_conf is not None else seed_confs[0]

    for pos in position_candidates:
        for orn in orientation_candidates:
            q = solve_collision_free_ik_seeded(
                pos,
                orn,
                collision_fn,
                seed_confs,
                attempts_random=attempts_random,
                pos_tol=pos_tol,
                reference_conf=reference_conf,
                max_reference_delta=max_reference_delta,
            )
            if q is None:
                continue
            score = DISTANCE_FN(ref, q)
            if score < best_score:
                best_score = score
                best = (q, pos, orn)

    return best if best is not None else (None, None, None)

# Checks whether a straight-line joint path from start to goal is collision-free.
# This cheap direct path is tried before BiRRT to avoid unnecessary planning when the path is simple.
def try_direct_path(
    q_start,
    q_goal,
    collision_fn,
):
  
    #Check a straight-line joint path before running BiRRT.
    path = [list(q_start)] + list(DIRECT_EXTEND_FN(q_start, q_goal))
    return path if all(not collision_fn(q) for q in path) else None


# Plans a collision-free joint path using BiRRT.
# It first checks a direct path, then runs several BiRRT attempts and returns the smoothed path with the lowest cost.
def plan_path(
    q_start,
    q_goal,
    collision_fn,
    attempts=2,
    max_time=1.5,
    smooth_time=0.25,
):
    #Plan a collision-free joint-space path using BiRRT.

    #A direct path is checked first. If that fails, several BiRRT attempts are
    #run and the smoothed path with the lowest weighted joint cost is returned.
    
    if q_goal is None:
        return None

    q_start, q_goal = list(q_start), list(q_goal)

    with RenderLock():
        saved_q = pp.get_joint_positions(env.ur5, ARM_JOINTS)
        try:
            if collision_fn(q_goal):
                return None

            direct = try_direct_path(q_start, q_goal, collision_fn)
            if direct is not None:
                return direct

            best_path, best_cost = None, np.inf
            state_id = p.saveState()
            try:
                        # Run independent BiRRT attempts and keep the lowest-cost smoothed path.
                for _ in range(attempts):
                    set_joint_positions(env.ur5, ARM_JOINTS, q_start)

                    path = pp.birrt(
                        q_start,
                        q_goal,
                        DISTANCE_FN,
                        SAMPLE_FN,
                        EXTEND_FN,
                        collision_fn,
                        max_iterations=pp.INF,
                        max_time=max_time,
                    )
                    if path is None:
                        continue

                    smoothed = pp.smooth_path(
                        path,
                        EXTEND_FN,
                        collision_fn,
                        max_smooth_iterations=60,
                        max_time=smooth_time,
                    )
                    cost = path_cost(smoothed)
                    if cost < best_cost:
                        best_cost, best_path = cost, smoothed
            finally:
                p.restoreState(stateId=state_id)
                p.removeState(state_id)

            return best_path
        finally:
            set_joint_positions(env.ur5, ARM_JOINTS, saved_q)


# Builds a straight Cartesian tool path and solves IK at each waypoint.
# Each waypoint is collision-checked, which makes short approach, lowering and retreat motions predictable.
def compute_cartesian_path(
    start_conf,
    start_pos,
    target_pos,
    orientation,
    collision_fn,
    cart_step=0.005,
    ik_pos_tol=0.02,
    max_segment_joint_jump=None,
):
    #Build a Cartesian straight-line tool path and solve IK at every waypoint.

    #This is used for short, controlled motions where Cartesian behaviour matters,
    #such as approaching the fruit, lowering to the drop point and retreating.

    start_pos = np.array(start_pos, dtype=float)
    target_pos = np.array(target_pos, dtype=float)
    total_dist = np.linalg.norm(target_pos - start_pos)
    if total_dist < 1e-8:
        return []

    segments = max(2, int(np.ceil(total_dist / cart_step)))
    path, prev_q = [], list(start_conf)

    for i in range(1, segments + 1):
        alpha = i / float(segments)
        interp = (1.0 - alpha) * start_pos + alpha * target_pos
        q_here = solve_collision_free_ik_seeded(
            interp,
            orientation,
            collision_fn,
            seed_confs=[prev_q, start_conf, HOME_JOINTS.tolist()],
            pos_tol=ik_pos_tol,
            reference_conf=prev_q,
            max_reference_delta=max_segment_joint_jump,
        )
        if q_here is None:
            return None
        path.append(q_here)
        prev_q = q_here

    return path


# Searches for the first valid grasp candidate.
# It loops through approach directions and roll angles until it finds a collision-free pre-grasp and Cartesian approach path.
def find_first_valid_grasp_candidate(
    collision_fn,
    search_timeout=6.0,
):
    #Search for a valid grasp pose and approach path.

    #The search loops through side-aware approach directions and gripper roll
    #angles, then accepts the first collision-free pre-grasp and approach path.

    start_time = time.time()
    q_start = pp.get_joint_positions(env.ur5, ARM_JOINTS)
    fruit_case = get_fruit_case(FRUIT_POS, TRUNK_CENTER)
    seed_confs = dedupe_confs([q_start, HOME_JOINTS.tolist()])

    approach_dirs = get_case_aware_approach_dirs(FRUIT_POS, TRUNK_CENTER)
    roll_candidates = get_case_aware_rolls(fruit_case)

    with RenderLock():
        state_id = p.saveState()
        try:
            # Try approach directions first, then roll angles for each direction.
            for approach_dir in approach_dirs:
                for roll in roll_candidates:
                    if time.time() - start_time > search_timeout:
                        return None

                    if fruit_case == "front":
                        pregrasp_clearance = 0.08
                        pregrasp_ref_delta = 2.0
                        approach_jump = 0.40
                        attempts_random = 0
                        pregrasp_reference = q_start
                    elif fruit_case == "right":
                        pregrasp_clearance = 0.14
                        pregrasp_ref_delta = 2.8
                        approach_jump = 0.55
                        attempts_random = 6
                        pregrasp_reference = q_start
                    else:
                        pregrasp_clearance = 0.08
                        pregrasp_ref_delta = 2.8
                        approach_jump = 0.55
                        attempts_random = 6
                        pregrasp_reference = q_start

                    pregrasp_pos, grasp_orn = compute_tool_pose_from_fruit(
                        FRUIT_POS,
                        approach_dir,
                        roll,
                        pregrasp_clearance,
                    )
                    grasp_pos, _ = compute_tool_pose_from_fruit(
                        FRUIT_POS,
                        approach_dir,
                        roll,
                        GRASP_CLEARANCE,
                    )

                    q_pre = solve_collision_free_ik_seeded(
                        pregrasp_pos,
                        grasp_orn,
                        collision_fn,
                        seed_confs=seed_confs,
                        attempts_random=attempts_random,
                        pos_tol=0.03,
                        reference_conf=pregrasp_reference,
                        max_reference_delta=pregrasp_ref_delta,
                    )
                    if q_pre is None:
                        continue

                    set_joint_positions(env.ur5, ARM_JOINTS, q_pre)
                    actual_pregrasp_pos, _ = pp.get_link_pose(env.ur5, TOOL_LINK)
                    actual_pregrasp_pos = np.array(actual_pregrasp_pos, dtype=float)

                    approach_path = compute_cartesian_path(
                        start_conf=q_pre,
                        start_pos=actual_pregrasp_pos,
                        target_pos=grasp_pos,
                        orientation=grasp_orn,
                        collision_fn=collision_fn,
                        cart_step=0.008,
                        ik_pos_tol=0.025,
                        max_segment_joint_jump=approach_jump,
                    )
                    if not approach_path:
                        continue

                    return {
                        "fruit_case": fruit_case,
                        "approach_dir": approach_dir,
                        "pregrasp_pos": actual_pregrasp_pos,
                        "grasp_pos": grasp_pos,
                        "grasp_orn": grasp_orn,
                        "q_pregrasp": q_pre,
                        "q_grasp": approach_path[-1],
                        "approach_path": approach_path,
                    }
        finally:
            p.restoreState(stateId=state_id)
            p.removeState(state_id)

    return None



# Commands the arm to one joint configuration and steps the simulator until it arrives or times out.
# The tolerance check prevents the code from moving on before the robot has reached the waypoint.
def execute_arm_conf(
    conf,
    max_velocity=0.45,
    conf_tol=0.02,
    max_steps=140,
):
    for i, j in enumerate(ARM_JOINTS):
        p.setJointMotorControl2(
            bodyIndex=env.ur5,
            jointIndex=j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=conf[i],
            maxVelocity=max_velocity,
            force=500,
        )
    for _ in range(max_steps):
        env.step()
        current_conf = pp.get_joint_positions(env.ur5, ARM_JOINTS)
        if np.linalg.norm(np.array(current_conf) - np.array(conf)) <= conf_tol:
            return True
    return False


# Executes a planned path one configuration at a time.
# Each waypoint is passed to execute_arm_conf so the simulation follows the planned trajectory.
def execute_path(
    path,
    max_velocity=0.45,
    conf_tol=0.02,
    max_steps_per_conf=140,
):
    if not path:
        return False
    # Send each planned waypoint to the simulated robot in order.
    for conf in path:
        execute_arm_conf(
            conf,
            max_velocity=max_velocity,
            conf_tol=conf_tol,
            max_steps=max_steps_per_conf,
        )
    return True


# Opens the gripper
def open_gripper(
    steps=200,
    target=0.0,
):
    for _ in range(steps):
        for j in GRIPPER_JOINTS:
            p.setJointMotorControl2(
                bodyIndex=env.ur5,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target,
                maxVelocity=1.0,
                force=1000,
            )
        env.step()


# Holds the gripper at its current joint positions.
# Used immediately after grasp contact so the fruit attachment remains stable before transport begins.
def hold_gripper_current(
    steps=80,
):
    targets = [p.getJointState(env.ur5, j)[0] for j in GRIPPER_JOINTS]
    for _ in range(steps):
        for i, j in enumerate(GRIPPER_JOINTS):
            p.setJointMotorControl2(
                bodyIndex=env.ur5,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=targets[i],
                maxVelocity=1.0,
                force=1000,
            )
        env.step()


# Closes the gripper and watches for contact with the fruit.
# Once contact is detected, the fruit is detached from the tree and attached to the gripper.
def close_gripper(
):
    attached = False
    # Keep closing for a fixed window while checking whether contact occurs.
    for _ in range(240):
        for j in GRIPPER_JOINTS:
            p.setJointMotorControl2(
                bodyIndex=env.ur5,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.8,
                maxVelocity=1.0,
                force=1000,
            )
        env.step()
        if not attached and fruit_constraint is not None:
            if get_gripper_contact() is not None:
                detach_fruit_from_tree()
                attach_fruit_to_gripper()
                attached = True
                hold_gripper_current(steps=80)
                break
    return attached



# Generates the benchmark fruit positions around the tree.
# It creates front, left and right cases with evenly spaced heights for systematic testing.
def generate_fruit_placements(
):
    z_start = 0.35
    z_end = 0.62
    n_per_side = 10

    heights = np.linspace(z_start, z_end, n_per_side)

    side_offsets = {
        "front": np.array([-FRUIT_RADIUS, 0.0]),
        "left": np.array([0.0, FRUIT_RADIUS]),
        "right": np.array([0.0, -FRUIT_RADIUS]),
    }

    placements = []
    # Build equal numbers of placements for each side of the tree.
    for side in ("front", "left", "right"):
        dx, dy = side_offsets[side]
        for z in heights:
            placements.append(np.array([
                TRUNK_CENTER[0] + dx,
                TRUNK_CENTER[1] + dy,
                z,
            ]))

    return placements


FRUIT_PLACEMENTS = generate_fruit_placements()
N_PLACEMENTS = len(FRUIT_PLACEMENTS)
REPS_PER_PLACEMENT = 1


# Resets the world for one benchmark cycle.
# It clears old constraints, returns the robot home, opens the gripper and attaches the fruit at the new position.
def reset_for_cycle(
    fruit_pos,
):
    global FRUIT_POS, FRUIT_ORN, fruit_constraint, grip_constraint

    for cid, name in [(grip_constraint, "grip_constraint"), (fruit_constraint, "fruit_constraint")]:
        if cid is not None:
            try:
                p.removeConstraint(cid)
            except Exception:
                pass
            globals()[name] = None

    disable_robot_fruit_collisions(disable=False)
    set_joint_positions(env.ur5, ARM_JOINTS, HOME_JOINTS.tolist())
    for _ in range(60):
        env.step()

    FRUIT_POS = np.array(fruit_pos, dtype=float)
    FRUIT_ORN = p.getBasePositionAndOrientation(env.fruit)[1]

    open_gripper(steps=80, target=0.0)
    attach_fruit_to_tree()


# Runs one full pick-and-place trial and records timing and failure information.
# The sequence is grasp search, pre-grasp planning, approach, grasp, retreat, carry planning, drop and return home.
def run_cycle(
    fruit_pos,
    cycle_idx,
    placement_idx,
    rep_idx,
):
    #Execute one complete pick-and-place cycle and return its result record.

    #The function records where the trial failed, how long planning took and
    #whether the fruit was successfully picked and placed
    global FRUIT_POS, chosen_predrop_orn

    t_start = time.time()

    rec = {
        "cycle": cycle_idx,
        "fruit_x": round(float(fruit_pos[0]), 4),
        "fruit_y": round(float(fruit_pos[1]), 4),
        "fruit_z": round(float(fruit_pos[2]), 4),
        "fruit_case": get_fruit_case(fruit_pos, TRUNK_CENTER),
        "success": False,
        "failure_stage": "none",
        "t_total_s": None,
        "t_plan_s": None,
        "t_grasp_search_s": None,
        "t_pregrasp_plan_s": None,
        "t_carry_plan_s": None,
    }

    def fail(stage):
        rec["failure_stage"] = stage
        rec["t_total_s"] = round(time.time() - t_start, 3)
        return rec

    try:
        reset_for_cycle(fruit_pos)

        print("  [1] Grasp search...")
        t0 = time.time()
        with RenderLock():
            collision_fn_free = build_collision_fn(include_tree=True)
            grasp_candidate = find_first_valid_grasp_candidate(
                collision_fn_free,
                search_timeout=6.0,
            )
        rec["t_grasp_search_s"] = round(time.time() - t0, 3)

        if grasp_candidate is None:
            return fail("grasp_search")
        print(f"  [1] Found: case={grasp_candidate['fruit_case']}")

        print("  [2] Planning to pre-grasp (BiRRT)...")
        t1 = time.time()
        with RenderLock():
            path_pregrasp = plan_path(
                pp.get_joint_positions(env.ur5, ARM_JOINTS),
                grasp_candidate["q_pregrasp"],
                collision_fn_free,
                attempts=3,
                max_time=3.0,
                smooth_time=0.6,
            )
        rec["t_pregrasp_plan_s"] = round(time.time() - t1, 3)

        if path_pregrasp is None:
            return fail("pregrasp_plan")
        rec["pregrasp_path_cost"] = round(path_cost(path_pregrasp), 4)

        execute_path(path_pregrasp, max_velocity=0.45)
        print("  [3] Approaching...")
        execute_path(
            grasp_candidate["approach_path"],
            max_velocity=0.10,
            conf_tol=0.02,
            max_steps_per_conf=160,
        )
        print("  [4] Closing gripper...")
        if not close_gripper():
            return fail("grasp_contact")

        fruit_attachment = create_attachment(env.ur5, TOOL_LINK, env.fruit)

        print("  [5] Retreating...")
        retreat_path = list(reversed(grasp_candidate["approach_path"]))
        if not retreat_path:
            return fail("retreat_empty")
        execute_path(
            retreat_path,
            max_velocity=0.10,
            conf_tol=0.02,
            max_steps_per_conf=160,
        )

        print("  [6] Planning carry path (BiRRT)...")
        t2 = time.time()
        with RenderLock():
            collision_fn_carry = build_collision_fn(
                attachments=[fruit_attachment],
                carrying=True,
                include_tree=True,
            )
            q_start = pp.get_joint_positions(env.ur5, ARM_JOINTS)
            _, current_carry_orn = pp.get_link_pose(env.ur5, TOOL_LINK)

            predrop_position_candidates = [
                DROP_POS + np.array([0.0, 0.0, h])
                for h in [0.10, 0.14, 0.18, 0.22]
            ]
            predrop_orientation_candidates = [
                current_carry_orn,
                ORIENTATION,
                p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2]),
                p.getQuaternionFromEuler([0, np.pi / 2, -np.pi / 2]),
                p.getQuaternionFromEuler([0, np.pi / 2, np.pi]),
            ]
            predrop_seeds = dedupe_confs([
                q_start,
                HOME_JOINTS.tolist(),
                grasp_candidate["q_pregrasp"],
                [0.0, -1.2, 1.8, -1.4, 0.0, 0.0],
            ])

            q_predrop, chosen_predrop_pos, chosen_predrop_orn = solve_collision_free_ik_multi_pose(
                predrop_position_candidates,
                predrop_orientation_candidates,
                collision_fn_carry,
                seed_confs=predrop_seeds,
                attempts_random=10,
                pos_tol=0.05,
            )
            if q_predrop is None:
                rec["t_carry_plan_s"] = round(time.time() - t2, 3)
                return fail("predrop_ik")

            path_predrop = plan_path(
                q_start,
                q_predrop,
                collision_fn_carry,
                attempts=4,
                max_time=4.0,
                smooth_time=0.8,
            )

            if path_predrop is None:
                print("  [6] Direct carry failed, trying overhead waypoint...")
                for wp in OVERHEAD_WAYPOINTS:
                    q_mid, _, _ = solve_collision_free_ik_multi_pose(
                        [wp],
                        [ORIENTATION, current_carry_orn],
                        collision_fn_carry,
                        seed_confs=[q_start, HOME_JOINTS.tolist()],
                        pos_tol=0.05,
                        reference_conf=q_start,
                        max_reference_delta=2.60,
                    )
                    if q_mid is None:
                        continue
                    leg1 = plan_path(
                        q_start,
                        q_mid,
                        collision_fn_carry,
                        attempts=3,
                        max_time=3.0,
                        smooth_time=0.6,
                    )
                    if leg1 is None:
                        continue
                    leg2 = plan_path(
                        q_mid,
                        q_predrop,
                        collision_fn_carry,
                        attempts=3,
                        max_time=3.0,
                        smooth_time=0.6,
                    )
                    if leg2 is None:
                        continue
                    path_predrop = list(leg1) + list(leg2[1:])
                    break

        rec["t_carry_plan_s"] = round(time.time() - t2, 3)

        if path_predrop is None:
            return fail("carry_plan")
        rec["carry_path_cost"] = round(path_cost(path_predrop), 4)

        execute_path(path_predrop, max_velocity=0.35)

        print("  [7] Lowering...")
        with RenderLock():
            drop_start_pos, _ = pp.get_link_pose(env.ur5, TOOL_LINK)
            drop_lower_path = compute_cartesian_path(
                start_conf=pp.get_joint_positions(env.ur5, ARM_JOINTS),
                start_pos=drop_start_pos,
                target_pos=DROP_POS,
                orientation=None,
                collision_fn=collision_fn_carry,
                cart_step=0.005,
                ik_pos_tol=0.03,
            )
        if drop_lower_path is None:
            return fail("drop_lower")

        execute_path(drop_lower_path, max_velocity=0.10)

        t_drop = time.time()
        print("  [8] Releasing...")
        open_gripper(steps=120, target=0.0)
        detach_fruit_from_gripper()
        for _ in range(120):
            env.step()

        rec["success"] = True
        rec["failure_stage"] = "none"
        rec["t_total_s"] = round(t_drop - t_start, 3)
        rec["t_plan_s"] = round(
            (rec["t_grasp_search_s"] or 0)
            + (rec["t_pregrasp_plan_s"] or 0)
            + (rec["t_carry_plan_s"] or 0),
            3,
        )

        collision_fn_home = build_collision_fn(include_tree=True)
        retreat_start, _ = pp.get_link_pose(env.ur5, TOOL_LINK)
        retreat_start = np.array(retreat_start, dtype=float)
        drop_retreat_path = compute_cartesian_path(
            start_conf=pp.get_joint_positions(env.ur5, ARM_JOINTS),
            start_pos=retreat_start,
            target_pos=retreat_start + np.array([0.0, 0.0, POSTDROP_RETREAT]),
            orientation=ORIENTATION,
            collision_fn=collision_fn_home,
            cart_step=0.005,
            ik_pos_tol=0.02,
            max_segment_joint_jump=0.40,
        )
        if drop_retreat_path:
            execute_path(drop_retreat_path, max_velocity=0.10)

        print("  [9] Returning home...")
        path_home = plan_path(
            pp.get_joint_positions(env.ur5, ARM_JOINTS),
            HOME_JOINTS.tolist(),
            build_collision_fn(include_tree=True),
            attempts=4,
            max_time=4.0,
            smooth_time=0.8,
        )
        if path_home:
            execute_path(path_home, max_velocity=0.40)
        open_gripper(steps=60, target=0.0)

    except Exception as exc:
        rec["failure_stage"] = f"exception:{str(exc)[:80]}"
        rec["t_total_s"] = round(time.time() - t_start, 3)
        print(f"  [!] Cycle {cycle_idx} exception: {exc}")

    finally:
        for cid, name in [(grip_constraint, "grip_constraint"), (fruit_constraint, "fruit_constraint")]:
            if cid is not None:
                try:
                    p.removeConstraint(cid)
                except Exception:
                    pass
                globals()[name] = None
        disable_robot_fruit_collisions(disable=False)

    return rec



CSV_FIELDS = [
    "cycle",
    "fruit_x", "fruit_y", "fruit_z", "fruit_case",
    "success", "failure_stage",
    "t_total_s", "t_plan_s",
    "t_grasp_search_s", "t_pregrasp_plan_s", "t_carry_plan_s",
]



# Main experiment loop.
# Runs every fruit placement, writes one CSV row per cycle and prints summary statistics at the end.
if __name__ == "__main__":
    CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "birrt_results.csv")
    all_results = []
    t_experiment_start = time.time()
    total_cycles = N_PLACEMENTS * REPS_PER_PLACEMENT

    with open(CSV_PATH, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()

        cycle_idx = 1
        # Outer loop chooses the fruit placement, inner loop repeats that placement.
        for placement_idx, fruit_pos in enumerate(FRUIT_PLACEMENTS):
            for rep_idx in range(1, REPS_PER_PLACEMENT + 1):
                print(f"\n{'=' * 65}")
                print(
                    f" Cycle {cycle_idx:>3}/{total_cycles}  |  "
                    f"Placement {placement_idx + 1:>2}/{N_PLACEMENTS}  |  "
                    f"Rep {rep_idx}/{REPS_PER_PLACEMENT}"
                )
                print(f" Fruit pos : {np.round(fruit_pos, 4)}")
                print(f" Fruit case: {get_fruit_case(fruit_pos, TRUNK_CENTER)}")
                print(f"{'=' * 65}")

                result = run_cycle(fruit_pos, cycle_idx, placement_idx, rep_idx)
                all_results.append(result)
                writer.writerow({k: result.get(k) for k in CSV_FIELDS})
                csv_file.flush()

                status = "✓" if result["success"] else f"✗ ({result['failure_stage']})"
                print(f"  → {status}  total={result['t_total_s']}s  plan={result['t_plan_s']}s")
                cycle_idx += 1

    t_total = round(time.time() - t_experiment_start, 1)
    n_ok = sum(1 for r in all_results if r["success"])
    times = [
        r["t_total_s"]
        for r in all_results
        if r["success"] and r["t_total_s"] is not None
    ]

    print(f"\n{'=' * 65}")
    print(" EXPERIMENT COMPLETE")
    print(f" Total wall time  : {t_total}s")
    print(f" Successful       : {n_ok} / {total_cycles} ({100 * n_ok // total_cycles}%)")
    print(f" Failed           : {total_cycles - n_ok}")
    if times:
        print(f" Cycle mean       : {round(np.mean(times), 2)}s")
        print(f" Cycle std        : {round(np.std(times), 2)}s")
        print(f" Cycle min / max  : {round(min(times), 2)}s / {round(max(times), 2)}s")
    print(f" CSV              : {CSV_PATH}")
    print(f"{'=' * 65}")

    while True:
        env.step()

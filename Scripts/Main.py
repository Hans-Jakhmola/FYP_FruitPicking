import pybullet as p
import time
import math
import pybullet_data
from Environment import environment
import pybullet_planning as pp
from pybullet_planning import INF
from pybullet_planning import connect, set_camera_pose, LockRenderer, wait_if_gui
from pybullet_planning import unit_pose, draw_pose
from pybullet_planning import compute_path_cost
import planner_2d_utils as mp_utils
from planner_2d_utils import create_aabb_box, get_aabb_center, draw_environment
from pybullet_planning import draw_aabb, BROWN
import numpy as np

env = environment()
env.initialise(gui=True)

SCALE = 0.13

raw_x_min, raw_x_max = -1.090,  0.210
raw_y_min, raw_y_max =  0.000,  7.000
raw_z_min, raw_z_max = -1.197,  0.403

raw_x_centre = (raw_x_min + raw_x_max) / 2.0
raw_y_centre = (raw_y_min + raw_y_max) / 2.0
raw_z_centre = (raw_z_min + raw_z_max) / 2.0

# Rotation [pi/2, 0, 0]: OBJ(x,y,z) -> world(x, -z, y)
world_dx =  raw_x_centre * SCALE
world_dy = -raw_z_centre * SCALE
world_dz =  raw_y_centre * SCALE

extent_x = 0.200   # trimmed from 0.240
extent_y = 0.193
extent_z = 0.864

tree_base   = np.array([0.8, -0.1, 0.0])
aabb_centre = np.array([
    tree_base[0] + world_dx - 0.03,  # shifted back from -0.05 to -0.03
    tree_base[1] + world_dy,
    tree_base[2] + world_dz
])

print(f"Tree AABB centre : {np.round(aabb_centre, 4)}")
print(f"Tree AABB extents: [{extent_x:.3f}, {extent_y:.3f}, {extent_z:.3f}]")
print(f"X box range: {aabb_centre[0]-extent_x/2:.4f} to {aabb_centre[0]+extent_x/2:.4f}")

tree_aabb = create_aabb_box(
    center=aabb_centre,
    extents=(extent_x, extent_y, extent_z)
)

draw_aabb(tree_aabb)

# ── Pass to planner as an obstacle ──────────────────────────────────────────
# obstacles = [tree_aabb]
# regions   = {'env': create_aabb_box(center=[0.65, 0.0, 0.5], extents=[0.6, 0.6, 1.2])}
# draw_environment(obstacles, regions)
# ─────────────────────────────────────────────────────────────────────────────

print('Environment ready - running simulation...')
while True:
    env.step()

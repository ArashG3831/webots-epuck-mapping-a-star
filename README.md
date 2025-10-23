# Cooperative Mapping & Path Planning in Webots (2× e-puck + A*)

Two e-pucks cooperatively map a maze with LIDAR + odometry, share an occupancy grid,
and a supervisor runs A* to the goal. Includes PID path-following and collision avoidance.

## Why it matters
- **Systems + control**: sensing → estimation → planning → control loop, end-to-end.
- **Reproducible**: one-command demo world; small, fast maps for reviewers.
- **Extensible**: plug in different planners (D*, RRT*) or controllers (LQR).

## Features
- 2 robots with independent controllers (pose estimation, obstacle avoidance).
- Shared **occupancy grid** with update fusion; simple outlier rejection.
- **A\*** planner on the supervisor; 4- or 8-connected; tie-breaking options.
- **PID** waypoint follower with anti-windup and lookahead; rate-limited commands.
- Metrics: map coverage %, path length, runtime, collisions (=0 in nominal demo).

## Quickstart
> **Tested on** Webots R2023+ and Python 3.10+

```bash
# 1) clone
git clone https://github.com/<you>/webots-epuck-mapping-a-star
cd webots-epuck-mapping-a-star

# 2) (optional) create venv & install deps for plotting
python -m venv .venv && . .venv/bin/activate  # windows: .venv\Scripts\activate
pip install -r requirements.txt

# 3) open world and run
#   Webots → File → Open World → worlds/maze_small.wbt
#   press "Run" ▶

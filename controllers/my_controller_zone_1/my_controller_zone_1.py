"""
e-puck2 autonomous mapping with encoder + compass + lidar
"""

from controller import Robot, Lidar, Compass
import numpy as np
from PIL import Image
import math

# ─── Map Config ───────────────────────────────────────────────────────
CELL = 0.01
SIZE = 200
MID = SIZE // 2
UNK, FREE, WALL = 0, 1, 2

L_MIN, L_MAX = 0.06, 1.5

# ─── Robot Config ──────────────────────────────────────────────────────
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.053

MAX_WHEEL = 6.28
FWD_SPEED = MAX_WHEEL * 0.5
TURN_SPEED = MAX_WHEEL * 0.5
BACK_SPEED = MAX_WHEEL * 0.3
FRONT_TH = 0.18

BACK_TIME = 0.25
STUCK_TIME = 3.0
RUNTIME = 1200000000000

# ─── Utils ─────────────────────────────────────────────────────────────
def bresenham(x0, y0, x1, y1):
    x0, y0, x1, y1 = map(int, map(round, (x0, y0, x1, y1)))
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1, -1)[x0 > x1], (1, -1)[y0 > y1]
    err = dx + dy
    pts = []
    while True:
        pts.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return pts
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy

def w2g(x, y, ox, oy):
    i = int(round((x - ox) / CELL)) + MID
    j = int(round((y - oy) / CELL)) + MID
    return (i, j) if 0 <= i < SIZE and 0 <= j < SIZE else (None, None)

def compass_heading(vec):
    rad = math.atan2(vec[1], vec[0])
    return rad if rad >= 0 else rad + 2 * math.pi

# ─── Main ──────────────────────────────────────────────────────────────
def main():
    robot = Robot()
    step = int(robot.getBasicTimeStep())

    # Devices
    lidar = robot.getDevice("Hokuyo URG-04LX")
    lidar.enable(step)
    lidar.enablePointCloud()

    compass = robot.getDevice("compass")
    compass.enable(step)

    left = robot.getDevice("left wheel motor")
    right = robot.getDevice("right wheel motor")
    for m in (left, right):
        m.setPosition(float('inf'))

    left_enc = robot.getDevice("left wheel sensor")
    right_enc = robot.getDevice("right wheel sensor")
    left_enc.enable(step)
    right_enc.enable(step)

    # Wait for all sensors to warm up
    for _ in range(10):
        robot.step(step)

    last_left = left_enc.getValue()
    last_right = right_enc.getValue()

    fov = lidar.getFov()
    beams = lidar.getHorizontalResolution() or lidar.getNumberOfPoints()

    ox, oy = 0.0, 0.0
    pose_x, pose_y = 0.0, 0.0
    last_pose = (0.0, 0.0)
    last_pose_t = 0.0

    elapsed = 0.0
    last_back = 0.0

    grid = np.zeros((SIZE, SIZE), np.uint8)

    # Start moving forward
    left.setVelocity(FWD_SPEED)
    right.setVelocity(FWD_SPEED)

    while robot.step(step) != -1:
        dt = step * 1e-3
        elapsed += dt

        # Read encoders
        cur_left = left_enc.getValue()
        cur_right = right_enc.getValue()
        dL = (cur_left - last_left) * WHEEL_RADIUS
        dR = (cur_right - last_right) * WHEEL_RADIUS
        last_left = cur_left
        last_right = cur_right

        # Get heading from compass
        theta = compass_heading(compass.getValues())

        # Update position (odometry)
        dxy = (dL + dR) / 2
        pose_x += dxy * math.cos(theta)
        pose_y += dxy * math.sin(theta)

        # Map current cell
        ci, cj = w2g(pose_x, pose_y, ox, oy)
        if ci is not None:
            grid[ci, cj] = FREE

        # LIDAR mapping
        rng = lidar.getRangeImage()
        for k, r in enumerate(rng):
            if math.isfinite(r) and L_MIN < r < L_MAX:
                ang = -fov / 2 + k * fov / (beams - 1)
                dx = r * math.cos(ang)
                dy = r * math.sin(ang)

                wx = pose_x + dx * math.cos(theta) - dy * math.sin(theta)
                wy = pose_y + dx * math.sin(theta) + dy * math.cos(theta)

                wi, wj = w2g(wx, wy, ox, oy)
                if wi is not None and ci is not None:
                    for fi, fj in bresenham(ci, cj, wi, wj)[:-1]:
                        grid[fi, fj] = FREE
                    grid[wi, wj] = WALL

        # ─── Obstacle Avoidance ───────────────────────────────────────
        reversing = elapsed - last_back < BACK_TIME
        if reversing:
            left.setVelocity(-BACK_SPEED)
            right.setVelocity(-BACK_SPEED)
        else:
            center = rng[beams//3: 2*beams//3]
            center_valid = [r for r in center if L_MIN < r < L_MAX]
            front = min(center_valid) if center_valid else float('inf')

            if front < FRONT_TH:
                left_min = min([r for r in rng[:beams//3] if L_MIN < r < L_MAX] or [float('inf')])
                right_min = min([r for r in rng[-beams//3:] if L_MIN < r < L_MAX] or [float('inf')])
                turn_left = right_min < left_min
                left.setVelocity(-TURN_SPEED if turn_left else TURN_SPEED)
                right.setVelocity(TURN_SPEED if turn_left else -TURN_SPEED)
                last_back = elapsed
            else:
                left.setVelocity(FWD_SPEED)
                right.setVelocity(FWD_SPEED)

        # ─── Stuck Detection ─────────────────────────────────────────
        if elapsed - last_pose_t > STUCK_TIME:
            dx = math.hypot(pose_x - last_pose[0], pose_y - last_pose[1])
            if dx < 0.04:
                last_back = elapsed - BACK_TIME
            last_pose = (pose_x, pose_y)
            last_pose_t = elapsed

        # Stop after RUNTIME
        if elapsed > RUNTIME:
            left.setVelocity(0)
            right.setVelocity(0)
            break

    # Save map
    img = np.zeros((SIZE, SIZE, 3), np.uint8)
    img[grid == UNK] = (128, 128, 128)
    img[grid == FREE] = (255, 255, 255)
    img[grid == WALL] = (0, 102, 255)
    Image.fromarray(np.flipud(img)).save("maze_map.png")
    print("Saved → maze_map.png")

if __name__ == "__main__":
    main()

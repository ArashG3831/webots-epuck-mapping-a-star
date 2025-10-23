import matplotlib.pyplot as plt

x_vals, y_vals = [], []

with open("map_points.txt", "r") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        x_vals.append(x)
        y_vals.append(y)

plt.figure(figsize=(8, 8))
plt.scatter(x_vals, y_vals, s=1, c='blue')
plt.title("Lidar-Based Maze Map")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.axis("equal")  # ensures 1m = 1m
plt.grid(True)
plt.show()

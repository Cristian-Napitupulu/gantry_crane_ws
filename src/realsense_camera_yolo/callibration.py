# Input three point s A, B, C and run this script to get the callibration matrix
# A is the center of container
# B is the right-left of container
# C is the top-bottom of container
# A, B, C should make a right angle
# Where A is the right angle
# And AB is the direction of the trolley
# For complete instruction, please refer to the README.md

import numpy as np
import matplotlib.pyplot as plt

# Points from measurement with realsense camera
point_A = np.array([0.018, -0.055, 0.550])
point_B = np.array([0.048, -0.055, 0.550])
point_C = np.array([0.020, -0.156, 0.558])
# Print points
for point, label in zip([point_A, point_B, point_C], ["A", "B", "C"]):
    print(f"Point {label}:", tuple(point))
print("")


# Create a vector from A to B
vector_AB = point_B - point_A
print("Vector AB:", tuple(vector_AB))

# Create a vector from A to C
vector_AC = point_C - point_A
print("Vector AC:", tuple(vector_AC))
print("")

# Calculate the cross product of AB and AC
# This will be the direction vector of the normal line
# The normal line is the line that is perpendicular to the plane of the container
# The normal line pierces through the center of the container (point  A)
normal_line_direction_vector = np.cross(vector_AB, vector_AC)
normal_line_direction_vector = normal_line_direction_vector / np.linalg.norm(
    normal_line_direction_vector
)

print("Normal line direction vector:", tuple(normal_line_direction_vector))
print(
    "Normal line equation: X = ({}, {}, {}) + t*({}, {}, {})".format(
        *point_A, *normal_line_direction_vector
    )
)
print("")

# Calculate the normal plane equation
# The plane equation is in the form of Ax + By + Cz + D = 0
# Where A, B, C are the components of vector AC
# And D is A * point_A_x + B * point_A_y + C * point_A_z
normal_plane_A = vector_AC[0]
normal_plane_B = vector_AC[1]
normal_plane_C = vector_AC[2]
normal_plane_D = -(
    normal_plane_A * point_A[0]
    + normal_plane_B * point_A[1]
    + normal_plane_C * point_A[2]
)
normal_plane_vector = np.array([normal_plane_A, normal_plane_B, normal_plane_C])

print(
    "Plane equation: {}x + {}y + {}z + {} = 0".format(
        normal_plane_A, normal_plane_B, normal_plane_C, normal_plane_D
    )
)
print("")

# Calculate line equation that parallel to vector AC and pierces through origin
# This line equation is in the form of (x, y, z) = t*(a, b, c)
# Where a, b, c are the components of vector AC
line_direction_vector = vector_AC

# Calculate which point on the line that is on the normal plane
# Substitute the line equation into the normal plane equation
# And solve for t
t = -normal_plane_D / (np.dot(normal_plane_vector, line_direction_vector))
# print("t: {}".format(t))

# Calculate the point on the line that is on the normal plane
point_on_line_on_plane = t * line_direction_vector
print("Point on line on normal plane:", tuple(point_on_line_on_plane))
print("")

# Find the closest point on the normal line to the point on the line on the plane
t = np.dot(normal_line_direction_vector, point_on_line_on_plane - point_A) / np.dot(
    normal_line_direction_vector, normal_line_direction_vector
)
# print("t: {}".format(t))

point_O = point_A + t * normal_line_direction_vector
print("Point O:", tuple(point_O))

# Calculate the distance between point O and origin
distance_O = np.linalg.norm(point_O)
print("Distance O to origin: {}".format(distance_O))
print("")

# Calculate the distance between point O and point A
distance_OA = np.linalg.norm(point_O - point_A)
print("Distance O to point A: {}".format(distance_OA))
print("")

# Print the callibration matrix
print("Callibration matrix:")
print("// Normal plane parameters from equation: Ax + By + Cz + D = 0")
print("#define NORMAL_PLANE_A {:.7f}".format(normal_plane_A))
print("#define NORMAL_PLANE_B {:.7f}".format(normal_plane_B))
print("#define NORMAL_PLANE_C {:.7f}".format(normal_plane_C))
print("#define NORMAL_PLANE_D {:.7f}".format(normal_plane_D))
print("// Normal line parameters from equation: (x, y, z) = (x_1, y_1, z_1) + t * (A, B, C)")
print("#define NORMAL_LINE_A {:.7f}".format(normal_line_direction_vector[0]))
print("#define NORMAL_LINE_B {:.7f}".format(normal_line_direction_vector[1]))
print("#define NORMAL_LINE_C {:.7f}".format(normal_line_direction_vector[2]))
print("// Trolley origin (x0, y0, z0)")
print("#define TROLLEY_ORIGIN_X {:.7f}".format(point_O[0]))
print("#define TROLLEY_ORIGIN_Y {:.7f}".format(point_O[1]))
print("#define TROLLEY_ORIGIN_Z {:.7f}".format(point_O[2]))
print("")

# Draw the normal line and the normal plane
# The normal line is the line that is perpendicular to the plane of the container
# The normal line pierces through the center of the container (point  A)
# The normal plane is the plane that is perpendicular to the vector AC
# The normal plane pierces through the center of the container (point  A)
# The normal line is on the normal plane
# Limit of the plot
limit = distance_OA

x_plane = np.linspace(-limit, limit, 10)
y_plane = np.linspace(-limit, limit, 10)
X_plane, Y_plane = np.meshgrid(x_plane, y_plane)
Z_plane = (
    (-normal_plane_A * X_plane - normal_plane_B * Y_plane - normal_plane_D)
    * 1.0
    / normal_plane_C
)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.plot_surface(X_plane, Y_plane, Z_plane, alpha=0.5)

normal_line_points = np.array(
    [point_A + t * normal_line_direction_vector for t in np.linspace(-10, 10, 100)]
)

ax.plot(
    normal_line_points[:, 0],
    normal_line_points[:, 1],
    normal_line_points[:, 2],
    color="g",
    linewidth=2.0,
    alpha=0.5,
)

ax.scatter(point_A[0], point_A[1], point_A[2], color="k", marker=".")
ax.scatter(point_B[0], point_B[1], point_B[2], color="k", marker=".")
ax.scatter(point_C[0], point_C[1], point_C[2], color="k", marker=".")
ax.scatter(point_O[0], point_O[1], point_O[2], color="r", marker="o")
ax.scatter(0, 0, 0, color="b", marker="x")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(-limit, limit)

ax.view_init(azim=-90, elev=-90)
plt.show()

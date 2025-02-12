import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D

# Container dimensions
CONTAINER_LENGTH = 0.325
CONTAINER_WIDTH = 0.134
CONTAINER_HEIGHT = 0.141

# Points from measurement with realsense camera
POINT_A = np.array([0.0038, 0.013, 0.4795]) # Center of the container
POINT_B = np.array([0.0345, 0.013, 0.48])   # Left or right of the container
POINT_C = np.array([0.0035, -0.0861, 0.5023])   # Front or back of the container
ACTUAL_CABLE_LENGTH = 0.4469


def print_points():
    """Print the coordinates of points A, B, and C."""
    for point, label in zip([POINT_A, POINT_B, POINT_C], ["A", "B", "C"]):
        print(f"Point {label}: {tuple(point)}")
    print("")


def vector_operations():
    """Perform vector calculations for the points."""
    vector_AB = POINT_B - POINT_A
    unit_vector_AB = vector_AB / np.linalg.norm(vector_AB)

    vector_AC = POINT_C - POINT_A
    unit_vector_AC = vector_AC / np.linalg.norm(vector_AC)

    normal_line_direction_vector = np.cross(vector_AB, vector_AC)
    normal_line_direction_vector /= np.linalg.norm(normal_line_direction_vector)

    angle_between_AB_AC = np.rad2deg(
        np.arccos(
            np.dot(vector_AB, vector_AC)
            / (np.linalg.norm(vector_AB) * np.linalg.norm(vector_AC))
        )
    )

    return (
        unit_vector_AB,
        unit_vector_AC,
        normal_line_direction_vector,
        angle_between_AB_AC,
    )


def calculate_plane_equation(normal_vector, point):
    """Calculate the plane equation Ax + By + Cz + D = 0."""
    A, B, C = normal_vector
    D = -np.dot(normal_vector, point)
    return A, B, C, D


def calculate_line_intersection(normal_vector, line_direction_vector, D):
    """Find the point on the line that lies on the plane."""
    t = -D / np.dot(normal_vector, line_direction_vector)
    return t * line_direction_vector


def find_closest_point_on_line(normal_vector, point_A, target_point):
    """Find the closest point on the normal line to the target point."""
    t = np.dot(normal_vector, target_point - point_A) / np.dot(
        normal_vector, normal_vector
    )
    return point_A + t * normal_vector


def print_calibration_matrix(
    normal_plane_vector, normal_line_direction_vector, point_O
):
    """Print the calibration matrix."""
    print("Calibration matrix:")
    print("// Normal plane parameters from equation: Ax + By + Cz + D = 0")
    for i, label in enumerate(["A", "B", "C", "D"]):
        print(f"#define NORMAL_PLANE_{label} {normal_plane_vector[i]:.7f}")
    print(
        "// Normal line parameters from equation: (x, y, z) = (x_1, y_1, z_1) + t * (A, B, C)"
    )
    for i, label in enumerate(["A", "B", "C"]):
        print(f"#define NORMAL_LINE_{label} {normal_line_direction_vector[i]:.7f}")
    print("// Trolley origin (x0, y0, z0)")
    for i, label in enumerate(["X", "Y", "Z"]):
        print(f"#define TROLLEY_ORIGIN_{label} {point_O[i]:.7f}")
    print("")


def draw_container(
    ax, center_top, size, unit_vector_AB, unit_vector_AC, unit_vector_OA
):
    """Draw the container on the 3D plot."""
    dx, dy, dz = size
    vertices = np.array(
        [
            center_top + dx / 2 * unit_vector_AC + dy / 2 * unit_vector_AB,
            center_top + dx / 2 * unit_vector_AC - dy / 2 * unit_vector_AB,
            center_top - dx / 2 * unit_vector_AC - dy / 2 * unit_vector_AB,
            center_top - dx / 2 * unit_vector_AC + dy / 2 * unit_vector_AB,
            center_top
            + dx / 2 * unit_vector_AC
            + dy / 2 * unit_vector_AB
            - dz * unit_vector_OA,
            center_top
            + dx / 2 * unit_vector_AC
            - dy / 2 * unit_vector_AB
            - dz * unit_vector_OA,
            center_top
            - dx / 2 * unit_vector_AC
            - dy / 2 * unit_vector_AB
            - dz * unit_vector_OA,
            center_top
            - dx / 2 * unit_vector_AC
            + dy / 2 * unit_vector_AB
            - dz * unit_vector_OA,
        ]
    )

    faces = [
        [vertices[j] for j in [0, 1, 2, 3]],
        [vertices[j] for j in [4, 5, 6, 7]],
        [vertices[j] for j in [0, 3, 7, 4]],
        [vertices[j] for j in [1, 2, 6, 5]],
        [vertices[j] for j in [0, 1, 5, 4]],
        [vertices[j] for j in [2, 3, 7, 6]],
    ]

    for vertice in vertices:
        ax.scatter(vertice[0], vertice[1], vertice[2], color="k", marker=".", alpha=0)
    ax.add_collection3d(
        Poly3DCollection(
            faces,
            facecolors=plt.get_cmap("tab20")(0),
            linewidths=1,
            edgecolors=plt.get_cmap("tab20")(2),
            alpha=0.15,
            label="Container",
        )
    )


def draw_axes(ax, origin, unit_vectors, length, axis_names, colors, label_prefix=""):
    """Draw the coordinate axes on the 3D plot."""
    origin = np.array(origin)  # Convert origin to a NumPy array
    unit_vectors = np.array(unit_vectors)  # Convert unit_vectors to a NumPy array

    for i, axis in enumerate(axis_names):
        ax.quiver(
            *origin, *unit_vectors[i], color=colors[i], length=length, normalize=True, alpha=0.5
        )
        ax.text(
            *(origin + unit_vectors[i] * length),
            f"{label_prefix}{axis}",
            color=colors[i],
        )


def set_axes_equal(ax):
    """Set the aspect ratio of 3D plot axes to be equal."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])
    return origin, radius


def distance_point_to_point(point1, point2):
    return np.linalg.norm(point1 - point2)


def plot_results():
    """Plot the results of the calculations and draw the 3D plot."""
    unit_vector_AB, unit_vector_AC, unit_vector_OA, angle_between_AB_AC = (
        vector_operations()
    )
    print(f"Angle between AB and AC: {angle_between_AB_AC:.2f} degrees")

    unit_vector_gravity = np.cross(unit_vector_AB, unit_vector_AC)
    unit_vector_gravity /= np.linalg.norm(unit_vector_gravity)

    dummy_point = POINT_A + unit_vector_gravity * ACTUAL_CABLE_LENGTH

    if distance_point_to_point(dummy_point, [0, 0, 0]) > distance_point_to_point(
        POINT_A, [0, 0, 0]
    ):
        unit_vector_gravity = -unit_vector_gravity

    point_O = POINT_A + unit_vector_gravity * ACTUAL_CABLE_LENGTH

    normal_plane_vector = unit_vector_AC
    normal_plane_A, normal_plane_B, normal_plane_C, normal_plane_D = (
        calculate_plane_equation(normal_plane_vector, POINT_A)
    )

    point_on_line_on_plane = calculate_line_intersection(
        normal_plane_vector, unit_vector_AC, normal_plane_D
    )
    # point_O = find_closest_point_on_line(
    #     unit_vector_OA, POINT_A, point_on_line_on_plane
    # )

    print_calibration_matrix(
        [normal_plane_A, normal_plane_B, normal_plane_C, normal_plane_D],
        unit_vector_OA,
        point_O,
    )

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(
        0, 0, 0, color=plt.get_cmap("tab20")(6), marker="o", s=30, label="Origin LiDAR"
    )
    ax.scatter(
        *point_O, color=plt.get_cmap("tab20")(4), marker="o", s=30, label="Pusat Troli"
    )

    # draw_axes(
    #     ax, [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 0.05, ["X", "Y", "Z"], "rgb"
    # )
    # draw_axes(
    #     ax,
    #     point_O,
    #     [-unit_vector_AB, -unit_vector_AC, -unit_vector_OA],
    #     0.05,
    #     ["X'", "Z'", "Y'"],
    #     "cym",
    # )

    ax.scatter(*POINT_A, color=plt.get_cmap("tab20")(8), marker=".", label="Titik A")
    ax.scatter(*POINT_B, color=plt.get_cmap("tab20")(10), marker=".", label="Titik B")
    ax.scatter(*POINT_C, color=plt.get_cmap("tab20")(12), marker=".", label="Titik C")

    ax.quiver(*POINT_A, *unit_vector_gravity, color="k", length=0.1, normalize=True, label="Vektor Arah Atas")

    draw_container(
        ax,
        POINT_A,
        [CONTAINER_LENGTH, CONTAINER_WIDTH, CONTAINER_HEIGHT],
        unit_vector_AB,
        unit_vector_AC,
        unit_vector_OA,
    )

    ax.plot(
        [POINT_A[0], 0],
        [POINT_A[1], 0],
        [POINT_A[2], 0],
        color=plt.get_cmap("tab20")(14),
        linestyle="-.",
        label="Jarak LiDAR ke titik A",
    )
    ax.plot(
        [POINT_A[0], point_O[0]],
        [POINT_A[1], point_O[1]],
        [POINT_A[2], point_O[2]],
        color=plt.get_cmap("tab20")(16),
        label=f"Tali Gantry {distance_point_to_point(POINT_A, point_O):.4f} m",
    )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect([1.0, 1.0, 1.0])

    _origin, _radius = set_axes_equal(ax)

    x_plane = np.linspace(_origin[0] - _radius * 1.5, _origin[0] + _radius * 1.5, 10)
    z_plane = np.linspace(_origin[2] - _radius * 1.5, _origin[2] + _radius * 1.5, 10)
    X_plane, Z_plane = np.meshgrid(x_plane, z_plane)
    Y_plane = (
        -normal_plane_A * X_plane - normal_plane_C * Z_plane - normal_plane_D
    ) / normal_plane_B
    Y_plane[
        (Y_plane < _origin[1] - _radius * 1.5) | (Y_plane > _origin[1] + _radius * 1.5)
    ] = np.nan
    ax.plot_surface(
        X_plane,
        Y_plane,
        Z_plane,
        color=plt.get_cmap("tab20")(18),
        alpha=0.2,
        label="Bidang Kerja Gantry",
    )

    # ax.view_init(elev=-90, azim=90, roll=0)
    ax.view_init(elev=178, azim=150, roll=14.5)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    print_points()
    plot_results()

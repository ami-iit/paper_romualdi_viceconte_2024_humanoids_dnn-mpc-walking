# file: utils.py
# Author: Giulio Romualdi
# License: BSD 3-Clause License

from PIL import ImageColor
from matplotlib.patches import Polygon
import numpy as np


def to_rgb(color, alpha=1.0):
    c = [v / 255.0 for v in ImageColor.getcolor(color, "RGB")]
    c.append(alpha)
    return c


def find_non_zero_chunks_with_threshold(input_vector, threshold):
    # Initialize variables to store the indices of chunks
    chunk_indices = []
    current_start_index = 0

    # Iterate through the vector
    for i in range(len(input_vector)):
        if abs(input_vector[i]) > threshold:
            if current_start_index == 0:
                current_start_index = i
        else:
            if current_start_index != 0:
                chunk_indices.append([current_start_index, i - 1])
                current_start_index = 0

    # Add the last chunk if it's not empty
    if current_start_index != 0:
        chunk_indices.append([current_start_index, len(input_vector) - 1])

    return np.array(chunk_indices)


def draw_arrow(
    axis, x, y, dx, dy, color="blue", arrow_length=0.1, label=None, width=0.001
):
    axis.arrow(
        x,
        y,
        dx,
        dy,
        head_width=arrow_length,
        head_length=arrow_length,
        fc=color,
        ec=color,
        label=label,
        width=width,
    )


def plot_rectangle_2D(axis, position, orientation, color, label=None, linewidth=0.5):
    # Define the local coordinates of the rectangle vertices (assuming it's centered at the origin)
    local_vertices = np.array(
        [
            [-0.08, -0.03],  # Vertex 1
            [0.08, -0.03],  # Vertex 2
            [0.08, 0.03],  # Vertex 3
            [-0.08, 0.03],  # Vertex 4
        ]
    )

    # Extract the rotation matrix from the quaternion [x, y, z, w]
    if len(orientation) == 4:
        q = orientation
    elif len(orientation) == 3:
        q = np.array([0, 0, np.sin(orientation[2] / 2), np.cos(orientation[2] / 2)])
    else:
        raise ValueError("Invalid orientation format")

    R = np.array(
        [
            [1 - 2 * (q[1] ** 2 + q[2] ** 2), 2 * (q[0] * q[1] - q[3] * q[2])],
            [2 * (q[0] * q[1] + q[3] * q[2]), 1 - 2 * (q[0] ** 2 + q[2] ** 2)],
        ]
    )

    # Apply the rotation and translation to the local vertices
    global_vertices = np.dot(local_vertices, R.T) + position[0:2]

    # Define the rectangle as a Polygon
    rectangle = Polygon(
        global_vertices,
        closed=True,
        edgecolor="black",
        facecolor=color,
        linewidth=linewidth,
        label=label,
    )

    # Add the rectangle to the axis
    axis.add_patch(rectangle)

# Description: This script is used to animate the steps and the CoM trajectory
# Author: Giulio Romualdi
# License: BSD 3-Clause License

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import h5py
from PIL import ImageColor
from matplotlib.patches import Polygon
import matplotlib.animation as animation
from dnn_mpc_plot.utils import (
    to_rgb,
    find_non_zero_chunks_with_threshold,
    draw_arrow,
    plot_rectangle_2D,
)
from rich.progress import track


# enable the antialiasing for the text
import matplotlib as mpl

from pathlib import Path


def animate(dataset_path: Path, animation_path: Path):
    animation_path.mkdir(parents=True, exist_ok=True)

    mpl.rcParams["text.antialiased"] = True

    class Index:
        def __init__(self, index=0, is_plot=False):
            self.index = index
            self.is_plot = is_plot

    external_push_indeces = [Index(index=1155), Index(index=1733), Index(index=2481)]
    limit = None

    controller_key = "cmw"

    matfile = h5py.File(
        str(dataset_path / "robot_logger_device_2024_07_09_12_48_52_c.mat"), "r"
    )
    left_foot = np.squeeze(
        matfile["robot_logger_device"][controller_key]["left_foot"]["position"][
            "desired"
        ]["data"]
    )
    right_foot = np.squeeze(
        matfile["robot_logger_device"][controller_key]["right_foot"]["position"][
            "desired"
        ]["data"]
    )
    left_foot_orientation = np.squeeze(
        matfile["robot_logger_device"][controller_key]["left_foot"]["orientation"][
            "desired"
        ]["data"]
    )
    right_foot_orientation = np.squeeze(
        matfile["robot_logger_device"][controller_key]["right_foot"]["orientation"][
            "desired"
        ]["data"]
    )
    left_foot_nominal = np.squeeze(
        matfile["robot_logger_device"][controller_key]["contact"]["left_foot"][
            "position"
        ]["nominal"]["data"]
    )
    right_foot_nominal = np.squeeze(
        matfile["robot_logger_device"][controller_key]["contact"]["right_foot"][
            "position"
        ]["nominal"]["data"]
    )

    if not limit:
        limit = left_foot.shape[0]

    external_push = np.squeeze(
        matfile["robot_logger_device"][controller_key]["external_wrench"]["filtered"][
            "data"
        ]
    )

    com = np.squeeze(
        matfile["robot_logger_device"][controller_key]["com"]["position"]["mpc_output"][
            "data"
        ]
    )
    com_mann = np.squeeze(
        matfile["robot_logger_device"][controller_key]["com"]["position"]["mann"][
            "data"
        ]
    )

    chunk_index_left = [
        Index(index=i)
        for i in find_non_zero_chunks_with_threshold(left_foot[:limit, 2], 0.00001)
    ]
    chunk_index_right = [
        Index(index=i)
        for i in find_non_zero_chunks_with_threshold(right_foot[:limit, 2], 0.00001)
    ]

    color_left = to_rgb("#7eb0d5", 0.9)
    color_right = color_left

    color_left_nominal = to_rgb("#b2e061", 0.9)
    color_right_nominal = color_left_nominal

    color_arrow = [0.6350, 0.0780, 0.1840]

    fig, ax = plt.subplots(1, figsize=(12, 11), facecolor="none")
    fig.patch.set_alpha(0.0)

    # set the axis equal
    ax.set_aspect("equal")

    # turn off axis spines
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)
    ax.set_frame_on(False)

    # plot com and com_mann
    (com_plot,) = ax.plot(
        [], [], label="CoM MPC", linewidth=5, antialiased=True, color=to_rgb("#ef9b20")
    )
    (com_mann_plot,) = ax.plot(
        [],
        [],
        label="CoM nominal",
        linewidth=5,
        antialiased=True,
        color=to_rgb("#9b19f5"),
    )

    plot_rectangle_2D(
        ax,
        left_foot_nominal[0, :],
        left_foot_orientation[0, :],
        color_left_nominal,
        linewidth=0.5,
        label="Nominal",
    )
    plot_rectangle_2D(
        ax,
        right_foot_nominal[0, :],
        right_foot_orientation[0, :],
        color_right_nominal,
        linewidth=0.5,
    )
    plot_rectangle_2D(
        ax,
        left_foot[0, :],
        left_foot_orientation[0, :],
        color_left,
        linewidth=0.5,
        label="Adjusted",
    )
    plot_rectangle_2D(
        ax, right_foot[0, :], right_foot_orientation[0, :], color_right, linewidth=0.5
    )

    ax.set_xlim(com[0, 0] - 0.3, com[0, 0] + 0.3)
    ax.set_ylim(com[0, 1] - 0.3, com[0, 1] + 0.3)

    def animate(i):
        com_plot.set_data(com[:i, 0], com[:i, 1])
        com_mann_plot.set_data(com_mann[:i, 0], com_mann[:i, 1])

        # Get the center of the image and set the limits
        center_x = ax.get_xlim()[0] + (ax.get_xlim()[1] - ax.get_xlim()[0]) / 2
        center_y = ax.get_ylim()[0] + (ax.get_ylim()[1] - ax.get_ylim()[0]) / 2

        # define the new center with the exponential filter
        new_center_x = center_x + 0.1 * (com[i, 0] - center_x)
        new_center_y = center_y + 0.1 * (com[i, 1] - center_y)

        ax.set_xlim(new_center_x - 0.3, new_center_x + 0.3)
        ax.set_ylim(new_center_y - 0.3, new_center_y + 0.3)

        # add all the patches to the axis
        for index in chunk_index_left:
            if i < index.index[1] or index.is_plot:
                continue

            plot_rectangle_2D(
                ax,
                left_foot_nominal[index.index[1] + 10, :],
                left_foot_orientation[index.index[1], :],
                color_left_nominal,
                linewidth=0.5,
            )

            plot_rectangle_2D(
                ax,
                left_foot[index.index[1], :],
                left_foot_orientation[index.index[1], :],
                color_left,
                linewidth=0.5,
            )

            index.is_plot = True

        for index in chunk_index_right:
            if i < index.index[1] or index.is_plot:
                continue

            plot_rectangle_2D(
                ax,
                right_foot_nominal[index.index[1] + 10, :],
                right_foot_orientation[index.index[1], :],
                color_right_nominal,
                linewidth=0.5,
            )

            plot_rectangle_2D(
                ax,
                right_foot[index.index[1], :],
                right_foot_orientation[index.index[1], :],
                color_right,
                linewidth=0.5,
            )

            index.is_plot = True

        for index in external_push_indeces:
            if i < index.index or index.is_plot:
                continue

            draw_arrow(
                ax,
                com[index.index, 0],
                com[index.index, 1],
                external_push[index.index, 0] / 4,
                external_push[index.index, 1] / 4,
                arrow_length=0.02,
                color=color_arrow,
            )

            index.is_plot = True

        return (
            com_plot,
            com_mann_plot,
        )

    ani = animation.FuncAnimation(
        fig, animate, frames=track(np.arange(0, limit, 3), description='Generation MPC animation...'), interval=30, blit=True
    )

    # put the legend outside the plot
    box = ax.get_position()
    ax.set_position([box.x0, box.y0 + box.height * 0.1, box.width, box.height * 0.9])

    plt.legend(fontsize="30", loc="upper center", ncol=2, bbox_to_anchor=(0.5, -0.05))

    ani.save(
        str(animation_path / "com_animation-c.mp4"),
        codec="png",
        dpi=100,
        bitrate=-1,
        savefig_kwargs={"transparent": True, "facecolor": "none"},
    )

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import h5py  # Import the h5py library
import scienceplots
from PIL import ImageColor
from dnn_mpc_plot import (
    find_non_zero_chunks_with_threshold,
    to_rgb,
    plot_rectangle_2D,
    draw_arrow,
)
from scipy.spatial.transform import Rotation as R
from pathlib import Path


def plot(dataset_path, plot_path):
    plt.style.use(["science", "ieee", "vibrant"])

    external_push_index = [1155, 1733, 2481]
    # external_push_index = []
    # limit = 8348
    limit = None
    controller_key = "cmw"

    matfile = h5py.File(
        str(dataset_path / "robot_logger_device_2024_07_09_12_48_52_c.mat"),
        "r",
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

    com_measured = np.squeeze(
        matfile["robot_logger_device"][controller_key]["com"]["position"]["measured"][
            "data"
        ]
    )

    dcom = np.squeeze(
        matfile["robot_logger_device"][controller_key]["com"]["velocity"]["measured"][
            "data"
        ]
    )

    dcom_mpc = np.squeeze(
        matfile["robot_logger_device"][controller_key]["com"]["velocity"]["mpc_output"][
            "data"
        ]
    )

    dcom_ik_input = np.squeeze(
        matfile["robot_logger_device"][controller_key]["com"]["velocity"]["ik_input"][
            "data"
        ]
    )

    com_mann = np.squeeze(
        matfile["robot_logger_device"][controller_key]["com"]["position"]["mann"][
            "data"
        ]
    )

    time = np.squeeze(
        matfile["robot_logger_device"][controller_key]["joints_state"]["positions"][
            "desired"
        ]["timestamps"]
    )
    time_measured = np.squeeze(
        matfile["robot_logger_device"]["joints_state"]["positions"]["timestamps"]
    )

    zmp_desired = np.squeeze(
        matfile["robot_logger_device"][controller_key]["zmp"]["desired"]["data"]
    )
    zmp_measured = np.squeeze(
        matfile["robot_logger_device"][controller_key]["zmp"]["measured"]["data"]
    )

    rotation = np.squeeze(
        matfile["robot_logger_device"][controller_key]["root_link"]["orientation"][
            "imu_estimator"
        ]["data"]
    )
    print(rotation.shape)

    time = time - time_measured[0]

    chunk_index_left = find_non_zero_chunks_with_threshold(
        left_foot[:limit, 2], 0.00001
    )
    chunk_index_right = find_non_zero_chunks_with_threshold(
        right_foot[:limit, 2], 0.00001
    )

    print(chunk_index_left)
    print(chunk_index_right)

    color_left = [0.4940, 0.1840, 0.5560, 0.8]
    color_left = to_rgb("#7eb0d5", 0.9)
    color_right = color_left

    color_left_nominal = [0.4660, 0.6740, 0.1880, 0.8]
    color_left_nominal = to_rgb("#b2e061", 0.9)
    color_right_nominal = color_left_nominal

    color_arrow = [0.6350, 0.0780, 0.1840]

    # we need to create a figure and axis object
    # the figure is a subplot of two rows and two columns but the first row spans both columns

    # set the figure height
    fig = plt.figure(figsize=(3.41, 4))
    gs = fig.add_gridspec(2, 2, height_ratios=(3, 1))
    ax = fig.add_subplot(gs[0, :])
    ax1 = fig.add_subplot(gs[1, 0])
    ax2 = fig.add_subplot(gs[1, 1])

    # add space between the subplots
    fig.subplots_adjust(hspace=0.1, wspace=0.1)

    # increase the size of ax

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    ax.set_xlim(0.4, 2.2)
    ax.set_ylim(-0.75, 0.25)
    ax.grid(True)

    # set axis equal aspect ratio
    ax.set_aspect("equal", "box")

    plot_rectangle_2D(
        ax, left_foot_nominal[0, :], left_foot_orientation[0, :], color_left_nominal
    )
    plot_rectangle_2D(
        ax, right_foot_nominal[0, :], right_foot_orientation[0, :], color_right_nominal
    )

    plot_rectangle_2D(ax, left_foot[0, :], left_foot_orientation[0, :], color_left)
    plot_rectangle_2D(ax, right_foot[0, :], right_foot_orientation[0, :], color_right)

    label = "Nominal"
    for i in range(chunk_index_left.shape[0]):
        plot_rectangle_2D(
            ax,
            left_foot_nominal[chunk_index_left[i, 1] + 10, :],
            left_foot_orientation[chunk_index_left[i, 1] + 10, :],
            color_left_nominal,
            label=label,
        )
        label = None

    for i in range(chunk_index_right.shape[0]):
        plot_rectangle_2D(
            ax,
            right_foot_nominal[chunk_index_right[i, 1] + 10, :],
            right_foot_orientation[chunk_index_right[i, 1] + 10, :],
            color_right_nominal,
        )

    label = "Adjusted"
    for i in range(chunk_index_left.shape[0]):
        plot_rectangle_2D(
            ax,
            left_foot[chunk_index_left[i, 1] + 10, :],
            left_foot_orientation[chunk_index_left[i, 1] + 10, :],
            color_left,
            label=label,
        )
        label = None

    for i in range(chunk_index_right.shape[0]):
        plot_rectangle_2D(
            ax,
            right_foot[chunk_index_right[i, 1] + 10, :],
            right_foot_orientation[chunk_index_right[i, 1] + 10, :],
            color_right,
        )

    # plot the com  and com_mann
    ax.plot(
        com[:limit, 0],
        com[:limit, 1],
        linewidth=1,
        label="CoM MPC",
        color=to_rgb("#ef9b20"),
    )
    ax.plot(
        com_mann[:limit, 0],
        com_mann[:limit, 1],
        linewidth=1,
        label="CoM nominal",
        color=to_rgb("#9b19f5"),
    )

    index_arrow = 0
    for i in external_push_index:
        draw_arrow(
            ax,
            com[i, 0],
            com[i, 1],
            external_push[i, 0] / 3,
            external_push[i, 1] / 3,
            arrow_length=0.05,
            color=color_arrow,
            label=label,
        )

        # # add a character to the arrow next to the head of the arrow with a small offset
        # # 1 is A 2 is B 3 is C and so on
        if index_arrow != 0:
            ax.text(
                com[i, 0] + external_push[i, 0] / 3 + 0.06,
                com[i, 1] + external_push[i, 1] / 3 - 0.02,
                chr(65 + index_arrow),
                fontsize=8,
                color=color_arrow,
            )
        else:
            ax.text(
                com[i, 0] + external_push[i, 0] / 3,
                com[i, 1] + external_push[i, 1] / 3 + 0.06,
                chr(65 + index_arrow),
                fontsize=8,
                color=color_arrow,
            )
        print(i)

        # draw a reference a reference frame placed at the CoM and oriented as the root link
        orientation_rot = R.from_euler("zyx", rotation[i, :], degrees=False)
        print("orientation", rotation[i, :])
        axes = np.array([[0.15, 0, 0], [0, 0.15, 0], [0, 0, 0.15]])
        rotated_axes = orientation_rot.apply(axes)

        print("rotated axes", rotated_axes)

        frame_color = to_rgb("#2f4f4f")

        if index_arrow != 0:
            draw_arrow(
                ax,
                com[i, 0],
                com[i, 1],
                rotated_axes[0, 0],
                rotated_axes[0, 1],
                arrow_length=0.02,
                color=frame_color,
            )
            draw_arrow(
                ax,
                com[i, 0],
                com[i, 1],
                rotated_axes[1, 0],
                rotated_axes[1, 1],
                arrow_length=0.02,
                color=frame_color,
            )
            # ax.text(
            #     com[i, 0] + 0.01,
            #     com[i, 1] + 0.05,
            #     "$B$",
            #     fontsize=8,
            #     color=frame_color,
            # )

        print("push force ", np.linalg.norm(external_push[i, :]) * 56)

        index_arrow += 1

        label = None

    # show legend
    ax.legend(ncol=2)

    # print the distance of the footsteps
    for i in range(chunk_index_right.shape[0]):
        print(
            "right foot distance",
            np.linalg.norm(
                right_foot[chunk_index_right[i, 1], :]
                - right_foot_nominal[chunk_index_right[i, 1] + 10, :]
            ),
        )

    for i in range(chunk_index_left.shape[0]):
        print(
            "left foot distance",
            np.linalg.norm(
                left_foot[chunk_index_left[i, 1], :]
                - left_foot_nominal[chunk_index_left[i, 1] + 10, :]
            ),
        )

    # rotate the  dcom
    dcom_rotated = np.zeros(dcom.shape)
    for i in range(dcom.shape[0]):
        # create a rotation matrix
        Rot = R.from_euler("zyx", rotation[i, :], degrees=False).inv()
        dcom_rotated[i, :] = Rot.apply(dcom[i, :])

    ax1.plot(time, dcom_rotated[:limit, 1], label="CoM x")

    # add a vertical line at the time of the push
    for i in external_push_index:
        ax1.axvline(time[i], color=color_arrow, linewidth=0.8, linestyle="--")

    # ax1.plot(time, -zmp_desired[:limit, 1] + zmp_measured[:limit, 1], label="ZMP measured x")
    # ax1.plot(time, com_measured[:limit, 1] - com[:limit, 1], label="CoM error x")
    # ax1.plot(time, dcom_mpc[:limit, 1], label="CoM velocity MPC x")
    ax1.set_xlim([61, 63])
    # add line at 0
    ax1.axhline(0, color="black", linewidth=0.5)
    ax1.set_xlabel("Time (s)")
    ax1.set_title("B")
    ax1.set_ylabel("${}^B \dot{p}_{CoM} - y$ (m/s)")

    ax2.plot(time, dcom_rotated[:limit, 1], label="CoM x")
    # ax2.plot(time, -zmp_desired[:limit, 1] + zmp_measured[:limit, 1], label="ZMP measured x")
    # ax2.plot(time, com_measured[:limit, 1] - com[:limit, 1], label="CoM error x")
    # ax2.plot(time, dcom_mpc[:limit, 1], label="CoM velocity MPC x")
    # add a vertical line at the time of the push dotting the line
    for i in external_push_index:
        ax2.axvline(time[i], color=color_arrow, linewidth=0.8, linestyle="--")

    ax2.set_xlim([68.5, 70.5])
    # add line at 0
    ax2.axhline(0, color="black", linewidth=0.5)
    ax2.set_title("C")

    # remove the y tick labels
    ax2.set_yticklabels([])

    ax2.set_xlabel("Time (s)")

    # move the legend outside the plot put it at the same hight of the x label
    # ax1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=4)

    # check if the folder figures exists if not create it
    plot_path.mkdir(parents=True, exist_ok=True)

    # save the figure
    plt.savefig(str(plot_path / "steps_and_com_controller.pdf"))

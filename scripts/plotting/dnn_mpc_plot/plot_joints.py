import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import h5py  # Import the h5py library
import scienceplots
from pathlib import Path


def plot(dataset_path: Path, plot_path: Path):
    plt.style.use(["science", "ieee", "vibrant"])

    joint_indices = [12, 13, 14, 18, 19, 20]
    joint_indices_measured = [
        4,
        5,
        6,
        7,
        8,
        9,
    ]
    #    10]
    joint_names = [
        "torso pitch",
        "torso roll",
        "torso yaw",
        "shoulder pitch",
        "shoulder roll",
        "shoulder yaw",
    ]

    matfile = h5py.File(
        str(dataset_path / "robot_logger_device_2024_07_03_15_08_33_p.mat"),
        "r",
    )
    controller_key = "cmw"

    joints_mann = np.squeeze(
        matfile["robot_logger_device"][controller_key]["joints_state"]["positions"][
            "mann"
        ]["data"]
    )
    joints_desired = np.squeeze(
        matfile["robot_logger_device"][controller_key]["joints_state"]["positions"][
            "desired"
        ]["data"]
    )
    joints_measured = np.squeeze(
        matfile["robot_logger_device"]["joints_state"]["positions"]["data"]
    )

    time = np.squeeze(
        matfile["robot_logger_device"][controller_key]["joints_state"]["positions"][
            "desired"
        ]["timestamps"]
    )
    time_measured = np.squeeze(
        matfile["robot_logger_device"]["joints_state"]["positions"]["timestamps"]
    )

    time_measured = time_measured - time[0]
    time = time - time[0]

    # create a subplot with 2 rows and 5 columns and set the size of the figure

    fig, axs = plt.subplots(3, 2, sharex=True, sharey=False)
    # add spacing between the subplots
    fig.subplots_adjust(hspace=0.45, wspace=0.45)

    # plot all the joints
    for i in range(3):
        for j in range(2):
            index = i * 2 + j

            # plot the joint with a given thickness
            axs[i][j].plot(
                time,
                joints_mann[:, joint_indices[index]],
                label="postural",
                linewidth=0.5,
            )
            # axs[i][j].plot(time, joints_desired[:, joint_indices[index]], label="desired", linewidth=0.5)
            axs[i][j].plot(
                time_measured,
                joints_measured[:, joint_indices_measured[index]],
                label="measured",
                linewidth=0.5,
            )
            # set the title
            axs[i][j].set_title(joint_names[index])
            # set the x limits
            axs[i][j].set_xlim([time[0], time[-1]])

    fig.supxlabel("time (s)")
    fig.supylabel("joint angle (rad)")

    # move the legend outside the plot put it at the same hight of the x label
    plt.legend(bbox_to_anchor=(-1.1, 4.2), loc="lower left", ncol=2)

    # check if the folder figures exists if not create it
    plot_path.mkdir(parents=True, exist_ok=True)

    # save the figure
    plt.savefig(str(plot_path / "joints_postural.pdf"))

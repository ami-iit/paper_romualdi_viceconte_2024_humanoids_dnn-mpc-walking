import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import h5py  # Import the h5py library
import scienceplots
import numpy as np

import pickle
from pathlib import Path


def plot(dataset_path: Path, plot_path: Path, load_from_wandb: bool = False):

    plt.style.use(["science", "ieee", "vibrant"])

    joint_list = [
        "l_hip_pitch",
        "l_hip_roll",
        "l_hip_yaw",
        "l_knee",
        "l_ankle_pitch",
        "l_ankle_roll",
    ]

    if load_from_wandb:
        import wandb

        run = wandb.Api().run("ami-iit/kf_autotuner/fs7rf6nj")
        history = run.scan_history()
        objectives = {}
        for joint in joint_list:
            print(joint)
            objectives[joint] = [
                row[joint + "  /best"]
                for row in history
                if row[joint + "  /best"] is not None
            ]
        with open(str(dataset_path / "ga_wandb_history.pkl"), "wb") as f:
            pickle.dump(objectives, f)
    else:
        with open(str(dataset_path / "ga_wandb_history.pkl"), "rb") as f:
            objectives = pickle.load(f)

    # plot the objectives for each joint on the same plot
    fig, ax = plt.subplots(1, sharex=True, sharey=False, figsize=(3.41, 1.4))
    for joint in joint_list:
        label = joint.replace("l_", "").replace("_", " ")
        ax.plot(np.array(objectives[joint]), label=label)
    ax.set_xlabel("GA Iteration")
    ax.set_ylabel("Objective")

    # move the legend outside the plot on the the top center
    plt.legend(
        bbox_to_anchor=(0, 1.02, 1, 0.2),
        loc="lower left",
        mode="expand",
        borderaxespad=0,
        ncol=3,
    )

    # check if the folder figures exists if not create it
    plot_path.mkdir(parents=True, exist_ok=True)

    # save the figure
    plt.savefig(str(plot_path / "ka_ga.pdf"))

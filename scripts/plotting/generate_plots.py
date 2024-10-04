from pathlib import Path

from dnn_mpc_plot import (
    plot_ga_kf_autotuning,
    plot_joints,
    plot_steps_and_com_controller,
    plot_steps_and_com_planner,
)


def main():
    current_file_path = Path(__file__).resolve()
    dataset_path = current_file_path.parent / "dataset" / "paper_experiments"
    plot_path = current_file_path.parent / "figures"

    plot_ga_kf_autotuning.plot(dataset_path, plot_path, load_from_wandb=False)
    plot_joints.plot(dataset_path, plot_path)
    plot_steps_and_com_controller.plot(dataset_path, plot_path)
    plot_steps_and_com_planner.plot(dataset_path, plot_path)


if __name__ == "__main__":
    main()

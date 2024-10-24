from pathlib import Path

from dnn_mpc_plot import (
    animate_plot_steps_and_com_planner,
    animate_plot_steps_and_com_controller,
)


def main():
    current_file_path = Path(__file__).resolve()
    dataset_path = current_file_path.parent.parent / "dataset" / "paper_experiments"
    animation_path = current_file_path.parent / "animations"

    animate_plot_steps_and_com_controller.animate(dataset_path, animation_path)
    animate_plot_steps_and_com_planner.animate(dataset_path, animation_path)


if __name__ == "__main__":
    main()

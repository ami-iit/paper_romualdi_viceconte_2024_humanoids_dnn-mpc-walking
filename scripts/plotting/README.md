
# 📚 Plotting for Online DNN-driven Nonlinear MPC in Stylistic Humanoid Robot Walking with Step Adjustment

This directory contains the scripts used to create the plots presented in the paper. The plots have been generated using the dataset stored in [Hugging Face 🤗](https://huggingface.co/datasets/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking_dataset). Please make sure you also clone the submodules in the repository. 

The dataset will be downloaded automatically as a submodule. Please note that the Hugging Face dataset requires `git-lfs` to be installed.

If you have already cloned the repository, please run the following command:
```console
git submodule update --init --recursive
```
Otherwise, clone the repository with the following command:
```console
git clone --recurse-submodules https://github.com/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking.git
```

## Requirements
The scripts are written in `Python`. To install all the required dependencies, run the following command:
```console
conda env create -f environment.yml
```

## Usage
The following scripts are available:
- `generate_animations.py`: Generates the plots animations of the robot walking and plots the steps and CoM trajectories;
- `generate_plots.py`: Generates the plots presented in the paper.
  
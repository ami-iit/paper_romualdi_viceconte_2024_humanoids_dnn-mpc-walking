<h1 align="center">
Online DNN-Driven Nonlinear MPC for Stylistic Humanoid Robot Walking with Step Adjustment
</h1>

<div align="center">
Giulio Romualdi, Paolo Maria Viceconte, Lorenzo Moretti, Ines Sorrentino, Stefano Dafarra, Silvio Traversaro, and Daniele Pucci  
<br>  
<b>Paolo Maria Viceconte and Giulio Romualdi are co-first authors</b>
</div>
<br>

<div align="center">
📅 This paper has been accepted for publication at the <b>2024 IEEE-RAS International Conference on Humanoid Robots</b> (Humanoids), Nancy, France 🤖
</div>
<br>

<div align="center">
   <a href="https://arxiv.org/abs/2410.07849"><b>📚 Paper</b></a> &nbsp;&nbsp;&nbsp; 
   <a href="https://www.youtube.com/watch?v=x3tzEfxO-xQ"><b>🎥 Video</b></a> &nbsp;&nbsp;&nbsp; 
   <a href="https://github.com/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking/blob/main/misc/poster/humanoids-2024-dnn-mpc.pdf"><b>🖼️ Poster</b></a> &nbsp;&nbsp;&nbsp;  
   <a href="#reproducing-the-experiments"><b>🔧 Experiments</b></a> &nbsp;&nbsp;&nbsp; 
   <a href="https://sites.google.com/view/dnn-mpc-walking/home-page"><b>🌐 Website</b></a> &nbsp;&nbsp;&nbsp; 
   <a href="https://huggingface.co/datasets/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking_dataset"><b>📂 Dataset</b></a> 
</div>

<br>

https://github.com/user-attachments/assets/404c9af8-528e-43c2-abd2-138a98adfc04

<br>

## Reproducing the Experiments

You can reprocue the experiments in two ways: either with conda or pixi.

### Conda


To reproduce the experiments, we provide a conda environment for easy setup. Follow the steps below:

#### 1. Install the Environment
Run the following command to create the conda environment:

```bash
conda env create -f environment.yml
```

#### 2. Activate the Environment
Activate the newly created environment:

```bash
conda activate dnn-mpc-env
```

#### 3. Run the Simulation
Start the experiment with:

```bash
./run_simulation.sh
```

This script will:  
- Launch the Gazebo simulator  
- Start the YARP server (for simulator communication)  
- Initialize the DNN-driven MPC controller  

When prompted, type `y` and press `Enter` to start the simulation. The humanoid robot will begin walking, and you can observe its behavior in the Gazebo simulator.

⚠️ **Known Issue:** The Gazebo real-time factor is scaled by a factor of 10. This is due to the use of the MUMPS linear solver in the IPOPT docker image. Alternative solvers (e.g., MA27) are available but cannot be redistributed.


### Pixi

To run the experiments with pixi on Linux, just download the repo and run:

~~~
git clone https://github.com/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking
cd paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking
pixi run -e default run_simulation
~~~

This command will install all the dependencies, compiled the code and run the simulation. At that point, when prompted, type `y` and press `Enter` to start the simulation. The humanoid robot will begin walking, and you can observe its behavior in the Gazebo simulator.

⚠️ **Known Issue:** The Gazebo real-time factor is scaled by a factor of 10. This is due to the use of the MUMPS linear solver in the IPOPT docker image. If you have have access to a Coin-HSL license, you can use it following the instructions in the following, to reduce the Gazebo real-time factor scaling from 10 to 2.

To run the simulation using the Coin-HSL's `ma97` solver, follow the following steps:

1. Go to https://licences.stfc.ac.uk/product/coin-hsl, and:
   - If you already have a license for Coin-HSL:
     - Go to https://licences.stfc.ac.uk/account/orders and find the coin-hsl order.
     - Download the coinhsl-2023.11.17.zip file and place it in the './coinhsl_src' folder of this repository.
   - If you do not have a license for Coin-HSL:
     - If you are an academic, request a license at https://licences.stfc.ac.uk/product/coin-hsl.
     - If you are not an academic, purchase a license at https://licences.stfc.ac.uk/product/coin-hsl.
     - Once your order is approved, download the coinhsl-2023.11.17.zip file and place it in the './coinhsl_src' folder.

Once the `coinhsl-2023.11.17.zip` archive is in the './coinhsl_src' folder, just run:

~~~
pixi run -e coinhsl run_simulation
~~~

This will execute the same steps of running `pixi run -e default run_simulation`, but additionally it will:
* compile `coinhsl` to be able to use the `ma97` linear solver,
* it will modify the configuration files to use `ma97`
* use a different Gazebo world model, to ensure a faster simulation.


---

## Maintainers

<table align="left">
    <tr>
        <td align="center">
            <a href="https://github.com/paolo-viceconte">
                <img src="https://github.com/paolo-viceconte.png" width="40" alt="Paolo Maria Viceconte"><br>
                👨‍💻 @paolo-viceconte
            </a>
        </td>
        <td align="center">
            <a href="https://github.com/GiulioRomualdi">
                <img src="https://github.com/GiulioRomualdi.png" width="40" alt="Giulio Romualdi"><br>
                👨‍💻 @GiulioRomualdi
            </a>
        </td>
    </tr>
</table>

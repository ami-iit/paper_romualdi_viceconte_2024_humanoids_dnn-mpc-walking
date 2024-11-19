<h1 align="center">
Online DNN-driven Nonlinear MPC for Stylistic Humanoid Robot Walking with Step Adjustment
</h1>

<div align="center">
Giulio Romualdi, Paolo Maria Viceconte, Lorenzo Moretti, Ines Sorrentino, Stefano Dafarra, Silvio Traversaro and Daniele Pucci <br> <br>
<b>Paolo Maria Viceconte and Giulio Romualdi are co-first authors</b>
</div>

<br>


<div align="center">
    📅 This paper has been accepted for publication at the 2024 IEEE-RAS International Conference on Humanoid Robots, (Humanoids) Nancy, France, 2024
 🤖
</div>
 
<div align="center">
   <a href="https://arxiv.org/abs/2410.07849"><b>📚 Paper</b> </a>  | <a href="https://github.com/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking/blob/main/misc/poster/humanoids-2024-dnn-mpc.pdf"><b> 🖼️ Poster</b> </a> |   <a href="#reproducing-the-experiments"><b>🔧 Experiments</b> </a> | <a href="https://sites.google.com/view/dnn-mpc-walking/home-page"><b>🌐 Website</b> </a>  | <a href="https://huggingface.co/datasets/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking_dataset"><b>📂 Dataset</b></a> 
</div>


## Reproducing the experiments

We support running the experiment via conda environment. To install the environment, please run the following commands:

```console
conda env create -f environment.yml
```

then activate the environment:

```console
conda activate dnn-mpc-env
```

Once the environment is activated, you can run the experiment by running the following command:

```console
./run_simulation.sh
```
The script will automatically run the Gazebo simulator, the yarp server (required for the communication with the simulator), and the DNN-driven MPC controller. Once everything is set up, the user needs to `y` and press `Enter` to start the simulation. The robot will start walking and the user can observe the results in the Gazebo simulator.

⚠️ Known issue: The gazebo real-time factor is scaled of factor 10. This is necessary since the linear solver used by IPOPT in the docker image is mumps. Unfortunately, other linear solvers (e.g. ma27) can be downloaded but not redistributed.

## Maintainers

<table align="left">
    <tr>
        <td><a href="https://github.com/paolo-viceconte"><img src="https://github.com/paolo-viceconte.png" width="40"></a></td>
        <td><a href="https://github.com/paolo-viceconte">👨‍💻 @paolo-viceconte</a></td>
    </tr>
    <tr>
        <td><a href="https://github.com/GiulioRomualdi"><img src="https://github.com/GiulioRomualdi.png" width="40"></a></td>
        <td><a href="https://github.com/GiulioRomualdi">👨‍💻 @GiulioRomualdi</a></td>
    </tr>
</table>

<h1 align="center">
Online DNN-Driven Nonlinear MPC for Stylistic Humanoid Robot Walking with Step Adjustment
</h1>

<div align="center">
Giulio Romualdi, Paolo Maria Viceconte, Lorenzo Moretti, Ines Sorrentino, Stefano Dafarra, Silvio Traversaro, and Daniele Pucci
<br>
<b>Co-first authors: Paolo Maria Viceconte and Giulio Romualdi</b>
</div>
<br>

<div align="center">
📅 Accepted for publication at the <b>2024 IEEE-RAS International Conference on Humanoid Robots</b> (Humanoids), Nancy, France 🤖
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

---

## Reproducing the Experiments

You can reproduce the experiments using **Docker**, **Conda**, or **Pixi**.

### Docker

Run the experiments via Docker for an isolated and reproducible environment.

1. Pull the Docker image:
   ```bash
   docker pull ghcr.io/ami-iit/dnn-mpc-walking-docker:latest
   ```

2. Launch the container:
   ```bash
   xhost +
   docker run -it --rm \
              --device=/dev/dri:/dev/dri \
              --env="DISPLAY=$DISPLAY" \
              --net=host \
              ghcr.io/ami-iit/dnn-mpc-walking-docker:latest
   ```

3. Wait for `Gazebo` to start and launch the experiment.

> ⚠️ **Known Issue:** The Gazebo real-time factor is scaled by a factor of 10 due to the MUMPS linear solver in the IPOPT Docker image. Alternative solvers (e.g., MA97) are available but cannot be redistributed.

---

### Conda

Follow these steps to set up the experiments using Conda:

1. Install the environment:
   ```bash
   conda env create -f environment.yml
   ```

2. Activate the environment:
   ```bash
   conda activate dnn-mpc-env
   ```

3. Compile the code:
   ```bash
   cd paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking
   mkdir build && cd build
   cmake ..
   make -j
   make install
   ```

4. Run the simulation:
   ```bash
   ./run_simulation.sh
   ```

> ⚠️ The Gazebo real-time factor is scaled by a factor of 10 due to the MUMPS linear solver.

---

### Pixi

To run the experiments with Pixi:

1. Clone the repository:
   ```bash
   git clone https://github.com/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking
   cd paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking
   ```

2. Run the simulation:
   ```bash
   pixi run -e default run_simulation
   ```

> **Using MA97 Solver (Optional):**
> If you have access to the Coin-HSL license, you can use the MA97 solver to improve performance:
> 1. Obtain the Coin-HSL archive (`coinhsl-2023.11.17.zip`) and place it in the `./coinhsl_src` folder.
> 2. Run:
>    ```bash
>    pixi run -e coinhsl run_simulation
>    ```

---

## Maintainers

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/paolo-viceconte">
        <img src="https://github.com/paolo-viceconte.png" width="80" alt="Paolo Maria Viceconte"><br>
        👨‍💻 Paolo Maria Viceconte
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/GiulioRomualdi">
        <img src="https://github.com/GiulioRomualdi.png" width="80" alt="Giulio Romualdi"><br>
        👨‍💻 Giulio Romualdi
      </a>
    </td>
  </tr>
</table>

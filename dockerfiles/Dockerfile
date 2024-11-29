# Use NVIDIA OpenGL base image for GUI access
FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04
LABEL org.opencontainers.image.title="DNN-MPC walking Docker Image"
LABEL org.opencontainers.image.description="Infrastructure for running a simulation of a humanoid robot walking using DNN-MPC architecture."
LABEL org.opencontainers.image.source="https://raw.githubusercontent.com/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking/refs/heads/main/dockerfiles/Dockerfile"
LABEL org.opencontainers.image.authors="Giulio Romualdi <giulio.romualdi@iit.it>"


# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/opt/conda/bin:$PATH"

# Install prerequisites
RUN apt-get update && \
    apt-get install -y wget bzip2 libx11-6 x11-utils x11-xserver-utils git git-lfs ffmpeg libsm6 libxext6 libgl1-mesa-dev libosmesa6-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Miniforge
RUN wget --quiet https://github.com/conda-forge/miniforge/releases/download/24.9.2-0/Mambaforge-24.9.2-0-Linux-x86_64.sh -O /tmp/miniforge.sh && \
    bash /tmp/miniforge.sh -b -p /opt/conda && \
    rm /tmp/miniforge.sh

# Create the Conda environment
RUN wget --quiet https://raw.githubusercontent.com/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking/refs/heads/main/environment.yml -O /tmp/environment.yml && \
    conda env create -f /tmp/environment.yml && \
    conda clean -afy

# Activate the environment by default
SHELL ["conda", "run", "-n", "dnn-mpc-env", "/bin/bash", "-c"]

# Create a folder /workspace and set it as the working directory
WORKDIR /workspace

# Clone the repository with the submodules compile it and install it in the conda environment
RUN git clone https://github.com/ami-iit/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking.git && \
    cd paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking && \
    git checkout v1.0.0 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/conda/envs/dnn-mpc-env .. && \
    make -j$(nproc) && \
    make install

RUN conda init && \
    echo "conda activate dnn-mpc-env" >> /root/.bashrc

CMD ["/bin/bash", "-i", "-c", "/workspace/paper_romualdi_viceconte_2024_humanoids_dnn-mpc-walking/run_simulation.sh"]

# Simulation Setup

## System Specs

* Ubuntu 22.04
* CUDA 12.2
* ROS 2 Humble

---

## ROS Installation

See the official installation instructions [here](https://docs.ros.org/en/humble/Installation.html). Install the recommended `ros-humble-desktop` package.

After installing ROS, install the ROS 2 Control packages used by the simulator:

```bash
sudo apt update

sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-forward-command-controller
```

Don't forget to source ROS before using it:

```bash
source /opt/ros/humble/setup.bash
```

You will probably want to add this to your `~/.bashrc` as well.

---

## Gazebo Installation

Follow the Gazebo installation guide [here](https://gazebosim.org/docs/latest/ros_installation/). Install **Gazebo Fortress**, since it is the version supported by ROS 2 Humble.

Next install the Gazebo ROS integration packages and common plugins:

```bash
sudo apt update

sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-image \
    ros-humble-gazebo-ros-pkgs
```

The simulator also uses camera sensors, so make sure the Gazebo sensor plugins are installed:

```bash
sudo apt install -y \
    gz-fortress \
    libgz-sim7-dev \
    libgz-sensors7-dev
```

The `ros_gz_bridge` package is used to bridge topics between Gazebo and ROS, including camera images, `/clock`, and other sensor topics.

---

## LeRobot and PyTorch

See the LeRobot installation guide [here](https://huggingface.co/docs/lerobot/en/installation) if you run into any issues.

Create a Conda environment:

```bash
conda create -n lerobot python=3.12 -y
conda activate lerobot
```

Clone LeRobot:

```bash
git clone https://github.com/huggingface/lerobot.git
cd lerobot
```

Install the required dependencies. Only the SmolVLA extension is needed.

```bash
conda install ffmpeg -c conda-forge

pip install -e ".[dataset,smolvla]"
```

By default, LeRobot installs the CPU version of PyTorch. If you have an NVIDIA GPU, uninstall it and install the CUDA build instead.

For CUDA 12.x systems:

```bash
pip uninstall -y torch torchvision torchaudio

# Install the CUDA build of PyTorch.
# CUDA 12.x runtimes are generally forward/backward compatible,
# so the CUDA 12.8 wheels work fine on CUDA 12.2.
pip install torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cu128
```

Verify that PyTorch can see your GPU:

```bash
python -c "
import torch
print('=' * 50)
print('✅ PyTorch Version:  ', torch.__version__)
print('✅ CUDA Available:   ', torch.cuda.is_available())
if torch.cuda.is_available():
    print('✅ GPU Device Name:  ', torch.cuda.get_device_name(0))
    print('✅ CUDA Version:     ', torch.version.cuda)
print('=' * 50)"
```

---

## Hugging Face

LeRobot uses the Hugging Face Hub for storing datasets and model checkpoints.

Install the CLI:

```bash
conda activate lerobot
pip install -U "huggingface_hub[cli]"
```

Login to your Hugging Face account:

```bash
hf auth login
```

You can verify with:
```bash
hf auth whoami
```

Once logged in, you can:

* Push trained policies to Hugging Face.
* Download pretrained checkpoints.
* Upload and download datasets.
* Train directly from datasets hosted on the Hub.

---

## Other LeRobot Dependencies

```bash
conda activate lerobot
pip install pynput
```

## Clone This Repository

Clone this repository into your ROS workspace:

```bash
git clone git@github.com:oberton31/LeRobot-ROS2-WS.git
```

Build the workspace:

```bash
cd ~/LeRobot-ROS2-WS
source /opt/ros/humble/setup.bash
colcon build
```

Finally, source the workspace:

```bash
source install/setup.bash
```

Again, adding this command to your `~/.bashrc` is recommended if you will be using the workspace regularly.

---

## Verify the Installation

At this point you should have:

* ROS 2 Humble installed.
* Gazebo Fortress installed.
* ROS ↔ Gazebo bridge working.
* LeRobot installed with GPU-enabled PyTorch.
* Logged into Hugging Face.
* The ROS workspace successfully built.

If all of the above completed without errors, you are ready to launch the simulator and begin collecting data or training policies.

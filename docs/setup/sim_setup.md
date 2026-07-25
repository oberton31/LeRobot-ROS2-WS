# Setting Up Simulation

## System Specs
- Ubuntu 22.04
- Cuda Version: 12.2
- ROS2 Humble

## ROS installation
See installation instructions [here](https://docs.ros.org/en/humble/Installation.html). Use the recommended ros-humble-desktop apt package.

**TODO: add instructions for installing ROS2 Control.**

## Gazebo Installation
See installation instructions [here](https://gazebosim.org/docs/latest/ros_installation/). You will want to install Gazebo Fortress.

**TODO: add instructions for sensors plugin, and gazeo ros bridge**

## LeRobot, PyTorch and HuggingFace
See installation guide [here](https://huggingface.co/docs/lerobot/en/installation) for questions. 

Create a conda env:
```bash
conda create -n lerobot python=3.12 -y
conda activate lerobot
```

Install lerobot
```bash
git clone https://github.com/huggingface/lerobot.git
cd lerobot
```

Install lerobot with pip, only the SmolVLA extension is necessary:
```bash
conda install ffmpeg -c conda-forge
pip install -e ".[smolvla]"
```

The LeRobot install will by default install torch without cuda. If possible, uninstall this version of torch, and reinstall torch with cuda. For example:
```bash
pip uninstall -y torch torchvision torchaudio

# reinstall PyTorch with CUDA 12.8 support (note that 12.x runtime as backward and forward compatible)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
```

Run the following command to verify installation:
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

TODO: ADD huggingface-cli so I can push/pull weights to huggingface (also easiest way to train the model)
##
#!/bin/bash

# default dataset configuration
HF_USERNAME="test"
DATASET_NAME="dry_run"

# parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --hf-username)
            HF_USERNAME="$2"
            shift 2
            ;;
        --dataset-name)
            DATASET_NAME="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

HF_REPO_ID="${HF_USERNAME}/${DATASET_NAME}"

echo "Using HuggingFace dataset: $HF_REPO_ID"

# terminal 1: Launch Gazebo + ROS
gnome-terminal --title="Gazebo" -- bash -c "
source /opt/ros/humble/setup.bash
source ../../../install/setup.bash
ros2 launch lerobot_gazebo gazebo_data_collection.launch.py
exec bash
"

# give Gazebo a chance to start
sleep 5

# terminal 2: Launch LeRobot recorder
gnome-terminal --title="LeRobot Recorder" -- bash -c "
cd ../lerobot_gazebo_robot

source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

python3 run_record.py \
    --robot.type=ros2_gazebo_robot \
    --teleop.type=keyboard_gazebo \
    --dataset.repo_id=$HF_REPO_ID \
    --dataset.push_to_hub=true \
    --dataset.num_episodes=10 \
    --dataset.episode_time_s=60

exec bash
"
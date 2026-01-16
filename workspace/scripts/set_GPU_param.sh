#!/usr/bin/bash

set -e

# Check if NVIDIA GPU is supported
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then 
    echo "NVIDIA GPU detected. Setting DOCKER_GPU_PARAM to '--gpu=all'"
    export DOCKER_GPU_PARAM="--gpu=all"
else
    # Check for Intel/AMD GPU via /dev/dri
    if [ -d "/dev/dri" ]; then
        echo "Non-NVIDIA GPU detected. Setting DOCKER_GPU_PARAM to '--device=/dev/dri'"
        export DOCKER_GPU_PARAM="--device=/dev/dri"
    else
        echo "No GPU devices detected. DOCKER_GPU_PARAM will be empty"
        export DOCKER_GPU_PARAM=""
    fi
fi

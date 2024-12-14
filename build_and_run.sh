#!/bin/bash
set -e

IMAGE_NAME="fishingrod_ws"
CONTAINER_NAME="${IMAGE_NAME}_container"

# Remove old container if it exists
echo "Removing old container (if any)..."
docker rm -f $CONTAINER_NAME || true

# Build the Docker image
echo "Building Docker image (using cache)..."
docker build -t $IMAGE_NAME .

# Uncomment the line below if you want to build with specific build arguments
# docker build --build-arg BASE_TAG=humble-desktop --build-arg ROS_DISTRO=humble -t $IMAGE_NAME .

# Run the Docker container with display access
echo "Running Docker container..."
# docker run -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --name $CONTAINER_NAME $IMAGE_NAME

# docker run -it --env="NVIDIA_VISIBLE_DEVICES=all" \
#   --env="NVIDIA_DRIVER_CAPABILITIES=all" \
#   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#   --volume="/usr/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu" \
#   --gpus all \
#   --name $CONTAINER_NAME $IMAGE_NAME

docker run --runtime=nvidia -it --gpus all \
  --name $CONTAINER_NAME $IMAGE_NAME



# Clean up unused Docker resources
echo "Cleaning up unused Docker resources..."
docker system prune -f

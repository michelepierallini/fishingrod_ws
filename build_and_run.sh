#!/bin/bash

IMAGE_NAME="fishingrod_ws"

echo "Removing old container (if any)..."
docker rm -f ${IMAGE_NAME}_container || true 

echo "Building Docker image (using cache)..."
docker build -t $IMAGE_NAME .

echo "Running Docker container..."
docker run -it --name ${IMAGE_NAME}_container $IMAGE_NAME
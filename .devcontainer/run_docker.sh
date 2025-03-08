#!/bin/bash

# 使用するイメージ名
IMAGE_NAME="doggy_ros1:latest"
CONTAINER_NAME="ros1_dev_container"

# X11 ディスプレイ設定 (GUIアプリ用)
xhost +local:root

# コンテナ実行
docker run -it --rm \
    --name $CONTAINER_NAME \
    --privileged \
    --net=host \
    --pid=host \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -e ROS_DOMAIN_ID=42 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:cached \
    -v /dev/dri:/dev/dri:cached \
    -v $(pwd)/..:/home/ws:cached \
    -w /home/ws \
    -u lin \
    $IMAGE_NAME \
    /bin/bash

# X11 設定を元に戻す
xhost -local:root

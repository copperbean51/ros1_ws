

# docker install ubuntu 24.04

sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt install docker-ce docker-ce-cli containerd.io
docker --version

sudo usermod -aG docker $USER
newgrp docker

# Catkin make build
catkin_make
source <your_ws>/devel/setup.bash

export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python2.7/dist-packages

roslaunch champ_config bringup.launch rviz:=true

roslaunch stanford_pupper_config bringup.launch rviz:=true


roslaunch champ_teleop teleop.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
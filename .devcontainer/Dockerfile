FROM tiryoh/ros-melodic-desktop:latest
ARG USERNAME=lin
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
    
RUN apt install -y tzdata
ENV TZ Asia/Tokyo

# base
RUN apt-get update
RUN apt-get install -y git vim curl wget

# ROS
RUN apt-get install -y python-rosdep \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-ecl-threads \
    ros-melodic-ecl-exceptions \
    ros-melodic-velodyne-gazebo-plugins \
    ros-melodic-ros-controllers \
    ros-melodic-joy
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

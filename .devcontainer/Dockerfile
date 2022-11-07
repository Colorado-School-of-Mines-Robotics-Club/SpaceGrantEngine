FROM osrf/ros:humble-desktop

# Update repositories
RUN apt-get update

# Python setup:
RUN apt-get install python3-pip -y

# Add ros user (with sudo)
RUN adduser --disabled-password --gecos '' ros
RUN adduser ros sudo

# Ensure sudo group users are not 
# asked for a password when using 
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to user ros
USER ros

# source ros2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

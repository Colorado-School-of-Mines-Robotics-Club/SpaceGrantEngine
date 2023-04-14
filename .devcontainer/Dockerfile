FROM osrf/ros:humble-desktop

# Update repositories
RUN apt-get update && apt-get upgrade -y

# Python setup:
RUN apt-get install python3-pip -y

# Install extra tools:
RUN apt-get install nano wget curl neofetch -y

# Add ros user (with sudo)
RUN adduser --disabled-password --gecos '' ros
RUN adduser ros sudo

# Ensure sudo group users are not 
# asked for a password when using 
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to user ros
USER ros

# Add python alias
RUN echo 'alias python="python3"' >> ~/.bashrc

# source ros2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install python packages from requirements files
COPY ../requirements-ci.txt /home/ros/
RUN pip3 install -r /home/ros/requirements-ci.txt
COPY ../requirements.txt /home/ros/
RUN pip3 install -r /home/ros/requirements.txt


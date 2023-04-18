FROM osrf/ros:humble-desktop

# Update repositories
RUN apt-get update && apt-get upgrade -y

# Install dependencies from installDeps.sh
COPY ../scripts/installDeps.sh /root/
COPY ../extern /root/extern
RUN bash /root/installDeps.sh

# Add ros user (with sudo)
RUN adduser --disabled-password --gecos '' ros && adduser ros sudo

# Ensure sudo group users are not 
# asked for a password when using 
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to user ros
USER ros

# Add python alias && source ros2
RUN echo 'alias python="python3"\nsource /opt/ros/humble/setup.bash' >> ~/.bashrc

FROM osrf/ros:humble-desktop
ENV ROS_DISTRO=humble

# Install tools
RUN apt update
RUN apt install -y sudo htop net-tools wget curl git gcc g++ make perl python3 python3-pip

# Add ros user (with sudo)
RUN adduser --disabled-password --gecos '' ros && adduser ros sudo

# Ensure sudo group users are not 
# asked for a password when using 
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to user ros
USER ros
WORKDIR /home/ros

# Add .local/bin to PATH (.bashrc)
RUN echo "export PATH=\"$HOME/.local/bin:$PATH\"" >> ~/.bashrc

# Add source ros2 to .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

COPY --chown=ros:ros ../scripts/install_deps.py ./
RUN ROS_DISTRO=$ROS_DISTRO python3 ./install_deps.py --apt_only

# Cleanup (as root)
USER root
RUN rm -rf ./install_deps.py
USER ros

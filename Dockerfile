# Use the MATLAB Runtime dependencies image for R2024b on Ubuntu 24.04 as base
FROM mathworks/matlab-runtime-deps:r2024b-ubuntu24.04

# Install necessary utilities and ROS2 Rolling Desktop packages
RUN apt-get update && apt-get install -y \
    wget \
    unzip \
    vim \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add the ROS2 repository key and repository for ROS2 Rolling
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS2 Rolling Desktop (change to "ros-rolling-desktop-full" if required)
RUN apt-get update && apt-get install -y ros-rolling-desktop && rm -rf /var/lib/apt/lists/*

# Source the ROS2 setup script for non-interactive shells
ENV ROS_DISTRO=rolling
SHELL ["/bin/bash", "-c"]

# Copy the MATLAB simulation script and project files into the container
WORKDIR /home/user/project
COPY AGV_simulation_multi_scenarios.m ./
COPY urdf/ ./urdf/
COPY launch/ ./launch/

# Expose port 11311 if needed for ROS communication
EXPOSE 11311

# Set the default command: run MATLAB in batch mode to execute the simulation script.
# Note: Ensure MATLAB Runtime is properly installed and licensed if needed.
CMD ["matlab", "-batch", "AGV_simulation_multi_scenarios"]

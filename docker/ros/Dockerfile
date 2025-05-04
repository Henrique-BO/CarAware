# ROS 2 Foxy base
FROM osrf/ros:foxy-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV TERM=xterm-color

# Install extra system dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    ca-certificates \
    git \
    build-essential \
    wget \
    python3-pip \
    python3-colcon-common-extensions \
    netcat \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install CARLA ROS Bridge dependencies
COPY install_dependencies.sh /tmp/ros-bridge/install-dependencies.sh
COPY requirements.txt /tmp/ros-bridge/requirements.txt
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    /tmp/ros-bridge/install-dependencies.sh"

# Create workspace
RUN mkdir -p /root/carla_ws/src
WORKDIR /root/carla_ws

# Setup entrypoints
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /root/carla_ws/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]

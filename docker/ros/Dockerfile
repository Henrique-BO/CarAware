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

# Install CARLA API
# COPY carla-0.9.13-py3.7-linux-x86_64.egg /tmp/carla-0.9.13-py3.7-linux-x86_64.egg

# # Install Python 3.7 manually
# RUN add-apt-repository ppa:deadsnakes/ppa && \
#     apt-get update && \
#     apt-get install -y python3.7 python3.7-dev python3.7-venv python3.7-tk \
#     && rm -rf /var/lib/apt/lists/*

# # Install pip for Python 3.7
# RUN wget https://bootstrap.pypa.io/pip/3.7/get-pip.py && \
#     python3.7 get-pip.py && \
#     rm get-pip.py

# # Create Python 3.7 virtual environment
# RUN python3.7 -m venv /opt/py37venv --system-site-packages --symlinks

# # Install necessary Python packages inside the venv
# RUN /bin/bash -c "source /opt/py37venv/bin/activate && \
#     pip install --upgrade pip setuptools wheel numpy"

# # Install CARLA ROS Bridge dependencies
# COPY install_dependencies.sh /tmp/ros-bridge/install-dependencies.sh
# COPY requirements.txt /tmp/ros-bridge/requirements.txt
# RUN /bin/bash -c "source /opt/py37venv/bin/activate && \
#     /tmp/ros-bridge/install-dependencies.sh"

# Create workspace
RUN mkdir -p /root/carla_ws/src
WORKDIR /root/carla_ws

# Setup entrypoints
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /root/carla_ws/install/setup.bash" >> ~/.bashrc
RUN echo "alias py37='source /opt/py37venv/bin/activate'" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]

# Use the official TensorFlow 1.15 GPU image
FROM tensorflow/tensorflow:1.15.0-gpu-py3

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install necessary dependencies
RUN apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/3bf863cc.pub
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    ca-certificates \
    git \
    build-essential \
    wget \
    ffmpeg \
    libsm6 \
    libxext6 \
    mesa-utis \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Install Python 3.7 (manually, since Ubuntu 22.04 doesn’t include it)
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.7 python3.7-dev python3.7-venv python3.7-tk

# Install pip
RUN wget https://bootstrap.pypa.io/pip/3.7/get-pip.py && \
    python3.7 get-pip.py && \
    rm get-pip.py

# Set Python 3.7 as the default Python version
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1 && \
    update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1
RUN python3.7 -m pip install setuptools==65.5.0 pip==21 wheel==0.38.0

# Install Python packages
COPY requirements.txt /tmp/requirements.txt
RUN python3.7 -m pip install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /app

CMD ["/bin/bash"]

# Use uma imagem base do Ubuntu 18.04 (ROS Melodic)
FROM ubuntu:18.04

# Set non-interactive environment variable
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    git \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libudev-dev \
    libssl-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxrandr-dev \
    libxi-dev \
    gfortran \
    libudev-dev \
    libdrm-dev \
    libgl1-mesa-dev \
    mesa-common-dev \
    python3 \
    python3-dev \
    python3-pip \
    gnupg2 \
    lsb-release \
    libglu1-mesa-dev\
    ca-certificates\
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt upgrade -y
# Istalando ffmpeg
RUN apt-get install ffmpeg -y

# Install pyenv
RUN git clone https://github.com/pyenv/pyenv.git ~/.pyenv
ENV HOME="/root"
ENV PYENV_ROOT="$HOME/.pyenv"
ENV PATH="$PYENV_ROOT/bin:$PATH"
RUN echo 'eval "$(pyenv init --path)"' >> ~/.bashrc

# Install Python 3.7.6 using pyenv
RUN /bin/bash -c "source ~/.bashrc && pyenv install 3.7.6 && pyenv global 3.7.6"

# Upgrade pip
RUN /bin/bash -c "source ~/.bashrc && python3 -m pip install --upgrade pip"



# # Instalando opencv
# RUN pip3 install opencv-python
# # Instalando rclpy
# RUN pip3 install roslibpy
# # Instalando o numpy
# RUN pip3 install numpy

# Atualize os pacotes e instale o curl
RUN apt-get update && apt-get install -y curl && rm -rf /var/lib/apt/lists/*

# Configure seu sources.list para o ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Adicione as chaves do ROS
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Atualize novamente os pacotes
RUN apt-get update

# Instale o ROS Melodic Desktop Full
RUN apt-get install -y ros-melodic-desktop-full

# Configure o ambiente ROS
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# Instale dependências para a construção de pacotes
RUN apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Inicialize rosdep
RUN apt-get install -y python-rosdep
RUN rosdep init
RUN rosdep update

# Crie um workspace chamado "vision"
RUN mkdir -p ~/vision_ws/src
WORKDIR /root/vision_ws

# Defina a variável de ambiente CATKIN_SHELL
ENV CATKIN_SHELL bash

# Execute catkin_make no workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# Install colcon (opcional)
# RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && python -m pip install colcon-common-extensions"

# Reset environment variable
ENV DEBIAN_FRONTEND=dialog

# Install colcon and run colcon build in the "vision_ws" directory
# RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && colcon build"

# Defina o comando padrão para inicializar diretamente no workspace "vision"
CMD ["/bin/bash", "-c", "source ~/vision_ws/devel/setup.bash && exec /bin/bash"]
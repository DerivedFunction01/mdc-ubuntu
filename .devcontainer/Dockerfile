# Start with Ubuntu 22.04 base image
FROM mcr.microsoft.com/devcontainers/base:jammy

# Set environment variables for locale
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Avoid interactive prompts during package install
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies and set locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    wget \
    udev \
    git-lfs \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Python and usbutils
RUN apt-get install -y python3 python3-pip python3-venv python3-dev usbutils && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Add ROS2 apt source dynamically based on GitHub release
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

# Update apt and install ROS2 Humble Desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \    
    && rm -rf /var/lib/apt/lists/*

# Add additional addons for ROS2 Humble
RUN apt-get update && apt-get install -y ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rtabmap-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-topic-tools \
    ros-humble-depthimage-to-laserscan



# -----------------------------
# ROS2 Tools (colcon, rosdep, etc.)
# -----------------------------
RUN pip3 install -U \
    colcon-common-extensions \
    rosdep \
    vcstool \
    argcomplete \
    pygame \
    pyserial
    # Enable argcomplete for bash (colcon/ros2 tab-completion)
RUN activate-global-python-argcomplete --user && \
    echo 'eval "$(register-python-argcomplete colcon)"' >> /root/.bashrc && \
    echo 'eval "$(register-python-argcomplete ros2)"' >> /root/.bashrc

# -----------------------------
# Azure Kinect SDK setup
# -----------------------------
# -----------------------------
# Azure Kinect udev rules
# -----------------------------
# Copy Kinect rules from project directory into container
COPY 99-k4a.rules /etc/udev/rules.d/99-k4a.rules

# Ensure plugdev group exists and root is part of it
RUN groupadd -f plugdev && usermod -aG plugdev root

# Reload udev rules (will apply at container runtime as well)
# RUN udevadm control --reload-rules && udevadm trigger

# Install necessary files
RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.2_amd64.deb
RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.2_amd64.deb
RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.2_amd64.deb
RUN wget http://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb

# Install debs
RUN dpkg -i libsoundio1_1.1.0-1_amd64.deb || true
RUN ACCEPT_EULA=Y dpkg -i libk4a1.4_1.4.2_amd64.deb  || true
RUN ACCEPT_EULA=Y dpkg -i libk4a1.4-dev_1.4.2_amd64.deb  || true
RUN dpkg -i k4a-tools_1.4.2_amd64.deb  || true

# Fix dependencies
RUN sudo apt update && \
 apt --fix-broken install -y

# Install tools for Maestro Servo Controller
COPY 99-pololu.rules /etc/udev/rules.d/99-pololu.rules
RUN wget https://www.pololu.com/file/0J315/maestro-linux-241004.tar.gz
RUN tar -xzvf maestro-linux-241004.tar.gz
RUN mv maestro-linux ~/maestro-linux
RUN apt-get install -y mono-runtime libmono-system-windows-forms4.0-cil
COPY MSC_config ~/maestro-linux/MSC_config

# Install VIM and vim-plug
RUN apt install vim -y
RUN curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim

# Copy the vimrc file and then quit
COPY .vimrc /root/
RUN vim +PlugInstall +q +q
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Clean up .deb files
RUN rm *.deb



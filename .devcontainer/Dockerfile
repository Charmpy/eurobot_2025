FROM osrf/ros:jazzy-desktop AS base

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN apt-get update && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && sudo rm -rf /var/lib/apt/lists/*

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################


RUN sudo apt-get update \
    && sudo apt-get install -y ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
    && sudo apt-get install -y ros-jazzy-tf-transformations  ros-jazzy-rqt-tf-tree \
    && sudo rm -rf /var/lib/apt/lists/*

RUN sudo apt update \
    && sudo apt install -y ros-jazzy-navigation2 \
    && sudo apt install -y ros-jazzy-nav2-bringup \
    && sudo apt install -y ros-jazzy-slam-toolbox \
    && sudo rm -rf /var/lib/apt/lists/*

FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
# Install gazebo python3-gz-transport13
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
  ros-jazzy-ros-gz   python3-gz-transport13 \
  && sudo apt-get install -y libgz-sim8-dev libgz-math8-dev libgz-common6-dev libgz-plugin3-dev \
  && sudo rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
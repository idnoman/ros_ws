ARG ROS_DISTRO="humble"
ARG PACKAGE_NAME="vision"
ARG ROS_WS_DIR="/ros_ws"

FROM ros:"${ROS_DISTRO}"-ros-base

ARG ROS_DISTRO
ARG PACKAGE_NAME
ARG ROS_WS_DIR

# ros-humble-* might be unnecessary
RUN apt-get update && apt-get install -y \
    build-essential \
    ros-humble-rmw-cyclonedds-cpp \
    v4l-utils \
    usbutils \
    libx11-6 x11-utils \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*


# pip installed Python dependencies
RUN pip install \
    cv_bridge \
    opencv-python \
    overrides

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=0

WORKDIR "${ROS_WS_DIR}"

COPY . "${ROS_WS_DIR}"/src/"${PACKAGE_NAME}"

RUN . /opt/ros/humble/setup.sh && colcon build

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ${ROS_WS_DIR}/install/setup.bash" >> ~/.bashrc

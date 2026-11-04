# Образ решения: ROS 2 Humble (desktop: rviz, rqt) + Nav2 + slam_toolbox + OpenCV.
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# dist-upgrade выравнивает версии ROS-пакетов базового образа с Nav2 (иначе lifecycle_manager
# падает без libdiagnostic_updater.so).
# packages.ros.org иногда не резолвится на раннерах CI: повторяем update, пока индекс ROS не скачается.
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries \
 && for i in 1 2 3 4 5; do apt-get update && apt-cache show ros-humble-navigation2 >/dev/null 2>&1 && break; \
      echo "apt: индекс ROS не получен, попытка $i"; sleep 20; done \
 && apt-get dist-upgrade -y && apt-get install -y --no-install-recommends \
      ros-humble-navigation2 \
      ros-humble-diagnostic-updater \
      ros-humble-nav2-bringup \
      ros-humble-nav2-mppi-controller \
      ros-humble-slam-toolbox \
      ros-humble-cv-bridge \
      ros-humble-rmw-fastrtps-cpp \
      python3-opencv \
      python3-numpy \
      python3-scipy \
      python3-pip \
      python3-colcon-common-extensions \
      git nano vim less \
 && rm -rf /var/lib/apt/lists/*

COPY docker/solution/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
 && mkdir -p /ros_ws/src \
 && echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ros_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "solution", "start.launch.py"]

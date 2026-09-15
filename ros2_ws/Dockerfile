FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

# Устанавливаем PCL, pcl_ros, pcl_conversions и инструменты сборки
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    zstd \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY src/ src/

RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && source install/setup.bash && exec bash"]

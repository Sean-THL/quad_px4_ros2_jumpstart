FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    ros-humble-std-srvs \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch v2.4.3 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/Micro-XRCE-DDS-Agent \
    && cmake -S /opt/Micro-XRCE-DDS-Agent -B /opt/Micro-XRCE-DDS-Agent/build \
    -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/Micro-XRCE-DDS-Agent/build --parallel 2 \
    && cmake --install /opt/Micro-XRCE-DDS-Agent/build \
    && ldconfig

WORKDIR /home/colcon_ws

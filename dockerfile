FROM osrf/ros:foxy-desktop

RUN apt-get update && \
apt-get install -y iputils-ping && \
apt-get install -y ros-foxy-control-msgs ros-foxy-serial-driver ros-foxy-ackermann-msgs ros-foxy-rosbridge-server ros-foxy-diagnostic-updater ros-foxy-test-msgs ros-foxy-urg-node python3-tk

COPY . ./f1tenth_ws/src/f1tenth_system 

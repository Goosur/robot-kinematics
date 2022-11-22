FROM osrf/ros:humble-desktop
ARG xauth_cookie
RUN apt update
RUN apt install -y ros-humble-dynamixel-sdk xauth
RUN xauth add ${xauth_cookie}

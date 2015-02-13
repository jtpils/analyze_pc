# Image based on Ubuntu 12.04 with all pre-requisites for footprints
#
# Copyright (c) 2015- Quanticare Technologies

# Base image
FROM ubuntu:12.04

# Maintainer/Author name and contact
MAINTAINER Savant Krishna <savant.2020@gmail.com>


# Installation start
RUN apt-get install -y wget ca-certificates
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
RUN apt-get update && apt-get install -y cmake git ros-hydro-desktop-full
RUN git clone https://github.com/sksavant/analyze_pc.git && RUN mkdir analyze_pc/build
RUN cd analyze_pc/build && cmake .. && make

# <todo> add other stuff

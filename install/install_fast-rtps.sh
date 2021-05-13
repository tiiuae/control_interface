#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

echo "$0: Installing Fast-RTPS"

GIT_PATH=~/git
ROS2_WORKSPACE_PATH=~/ros2_workspace

install_prerequisites(){
  echo 'Installing prerequisities'
  sudo apt update
  sudo apt install ros-foxy-foonathan-memory-vendor
  sudo apt install libasio-dev libtinyxml2-dev
  sudo apt install unzip
  echo 'Done'
}

install_java_and_gradle(){
  echo 'Installing java and gradle'
  sudo apt install openjdk-14-jdk
  sudo add-apt-repository ppa:cwchien/gradle
  sudo apt update
  sudo apt install gradle
}

install_fastdds(){
  if [ $ROS_DISTRO != 'foxy' ]; then
    source /opt/ros/foxy/setup.bash
  fi
  echo 'Installing Fast-DDS'
  cd $GIT_PATH
  [ ! -e Fast-DDS ] && git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.0 Fast-DDS
  [ ! -e 'Fast-DDS/build' ] && mkdir Fast-DDS/build
  cd Fast-DDS/build
  cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
  make -j$(nproc --all)
  sudo make install
  echo 'Done'
}

install_fastrtps_gen(){
  echo 'Installing Fast-RTPS-Gen'
  cd $GIT_PATH
  [ ! -e Fast-RTPS-Gen ] && git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 Fast-RTPS-Gen
  cd Fast-RTPS-Gen && gradle assemble && sudo gradle install
  echo 'Done'
}

install_prerequisites
install_java_and_gradle
install_fastdds
install_fastrtps_gen

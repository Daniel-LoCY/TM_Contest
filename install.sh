#!/bin/bash

apt install -y git python3-pip wget unzip 

wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb

wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb

wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb

mv *.deb /tmp

dpkg -i /tmp/*.deb

apt install k4a-tools -y

apt install --fix-broken

apt install k4a-tools -y

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

catkin_make
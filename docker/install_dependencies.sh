#!/bin/bash
set -e 
sudo apt-get update
# Install 
apt install -y libnanoflann-dev
git clone https://github.com/usdot-fhwa-stol/carma-msgs /root/cda_ws/src/carma-msgs && git clone -b route-server-develop https://github.com/usdot-fhwa-stol/navigation2 /root/cda_ws/src/navigation2
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=humble -y -r

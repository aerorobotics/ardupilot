#!/bin/bash -e

# Build the docker
echo "Building docker"
docker build . -t ardupilot

# Make sure submodules are up to date
echo "Updating submodules"
docker run --user $(id -u):$(id -g) -v `pwd`:/ardupilot ardupilot:latest git submodule update --init --recursive

# --- 
# Build CubeOrangePlus version of the code
echo "Configuring build"
docker run --user $(id -u):$(id -g) -v `pwd`:/ardupilot ardupilot:latest ./waf configure --board CubeOrangePlus --enable-opendroneid
# docker run --user $(id -u):$(id -g) -v `pwd`:/ardupilot ardupilot:latest ./waf configure --board Pixhawk4 --enable-opendroneid
# docker run --user $(id -u):$(id -g) -v `pwd`:/ardupilot ardupilot:latest ./waf configure --board CubeOrangePlus-SimOnHardWare --enable-opendroneid
# docker run --user $(id -u):$(id -g) -v `pwd`:/ardupilot ardupilot:latest ./waf copter

exit 0

# ----

# Start ArduCopter SITL
# docker run -it \
#     --net=host \
#     -v `pwd`:/ardupilot \
#     -u "$(id -u):$(id -g)" \
#     ardupilot:latest sim_vehicle.py -v ArduCopter

# exit 0

# ----

# Start the docker with command line
# docker run -it \
#     --net=host \
#     -v `pwd`:/ardupilot \
#     -u "$(id -u):$(id -g)" \
#     ardupilot:latest bash

# ----

# SITL-on-Hardware
#docker run -it \
#    --net=host \
#    -v `pwd`:/ardupilot \
#    -u "$(id -u):$(id -g)" \
#    ardupilot:latest \
#    ./libraries/SITL/examples/on-hardware/sitl-on-hw.py --board Pixhawk4 --vehicle copter

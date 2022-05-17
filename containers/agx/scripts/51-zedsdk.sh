#!/bin/bash

# cd /tmp/scripts

#This environment variable is needed to use the streaming features on Jetson inside a container
LOGNAME=root
DEBIAN_FRONTEND=noninteractive
apt-get update -y && apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https -y && \
    echo "# R32 (release), REVISION: 6.1" > /etc/nv_tegra_release

wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run https://download.stereolabs.com/zedsdk/3.7/l4t32.6/jetsons && \
    chmod +x ZED_SDK_Linux_JP.run ; su wagrandprix -c "./ZED_SDK_Linux_JP.run silent runtime_only" && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux_JP.run && \
    rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# wget https://download.stereolabs.com/zedsdk/3.7/l4t32.6/jetsons
# chmod +x ./jetsons
# ./jetsons -- silent
# 
# # changing ownership so that package can access the zed sdk
# chown -R wagrandprix: /usr/local/zed
# 
# export CUDA_BIN_PATH=/usr/local/cuda-10.2
# 
# distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
#       && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
#       && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
#             sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
#             sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
# 
# apt-get update
# apt-get install -y nvidia-docker2

# ln -s /usr/lib/aarch64-linux-gnu/tegra/libcuda.so /usr/lib/aarch64-linux-gpu/tegra/libcuda.so.0
# # linking because cmake looks for /usr/local/cuda while finding cuda, but we have .../cuda-10.2
# ln -s /usr/local/cuda-10.2 /usr/local/cuda

# # more linking to get packages to build - almost definitely a band-aid solution and needs to be fixed permanantly, probably connected to the cuda linking which should not have been necessary
# ln -s /usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 /usr/lib/aarch64-linux-gnu/libusb-1.0.so
# ln -s /usr/local/cuda-10.2/targets/aarch64-linux/lib/stubs/libcuda.so /usr/lib/aarch64-linux-gnu/libcuda.so.1
# ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so /usr/lib/aarch64-linux-gnu/libnvbuf_utils.so.1.0.0
# ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvrm.so /usr/lib/aarch64-linux-gnu/libnvrm.so
# ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvrm_graphics.so /usr/lib/aarch64-linux-gnu/libnvrm_graphics.so
# ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvddk_vic.so /usr/lib/aarch64-linux-gnu/libnvddk_vic.so
# ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvbuf_fdmap.so.1.0.0 /usr/lib/aarch64-linux-gnu/libnvbuf_fdmap.so.1.0.0
# ln -s /usr/lib/aarch64-linux-gnu/tegra/libnvos.so /usr/lib/aarch64-linux-gnu/libnvos.so

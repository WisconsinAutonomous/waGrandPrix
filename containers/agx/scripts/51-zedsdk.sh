#!/bin/bash

cd /tmp/scripts

apt install wget -y

wget https://download.stereolabs.com/zedsdk/3.7/l4t32.6/jetsons
chmod +x ./jetsons
./jetsons -- silent

# changing ownership so that package can access the zed sdk
chown -R wagrandprix: /usr/local/zed

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

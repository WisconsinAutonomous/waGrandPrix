#!/bin/bash

cd /tmp/scripts

apt install wget -y

wget https://download.stereolabs.com/zedsdk/3.7/l4t32.6/jetsons
chmod +x ./jetsons
./jetsons -- silent

# changing ownership so that package can access the zed sdk
chown -R wagrandprix: /usr/local/zed

# linking because cmake looks for /usr/local/cuda while finding cuda, but we have .../cuda-10.2
ln -s /usr/local/cuda-10.2 /usr/local/cuda

#!/bin/bash

apt-get update -qq
apt-get install -y software-properties-common
apt-add-repository -y ppa:astuff/kvaser-linux
apt-get update -qq
apt-get install kvaser-canlib-dev

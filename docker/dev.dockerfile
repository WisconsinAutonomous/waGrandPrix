FROM ros:foxy

LABEL maintainer="Aaron Young <aryoung5@wisc.edu"

ARG DEBIAN_FRONTEND=noninteractive
ARG ROSDISTRO=foxy

# Use mirrors instead of main server
RUN sed -i 's|deb http://.*ubuntu.com.* \(focal.*\)|deb mirror://mirrors.ubuntu.com/mirrors.txt \1|g' /etc/apt/sources.list

# Check for updates
RUN apt update && apt upgrade -y && apt install -y expect

# Install some packages
RUN apt install -y tmux vim ssh git git-lfs zsh python3-pip gxmessage nodejs npm libboost-all-dev
RUN apt install -y gnupg2 lsb-release curl 

# Install some ros stuff
RUN apt install -y ros-$ROSDISTRO-example-interfaces ros-$ROSDISTRO-cv-bridge ros-$ROSDISTRO-diagnostic-updater ros-$ROSDISTRO-image-transport ros-$ROSDISTRO-xacro

# Install hygen to create boilerplate code
RUN npm install -g hygen

# Install some python packages
RUN pip install numpy pandas matplotlib python-can autobahn tornado twisted Pillow

# Some weird stuff for bison
RUN pip install bson && pip install hyperopt && pip install hyperas && sudo pip uninstall bson && pip install pymongo

# Various arguments and user settings
ARG USERSHELL
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="/root/.${USERSHELL}rc"

# ROS Setup
RUN sed -i 's|source|#source|g' /ros_entrypoint.sh
RUN echo ". /opt/ros/$ROSDISTRO/setup.sh" >> $USERSHELLPROFILE
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash"

# Environment
ENV HYGEN_TMPLS=/root/waGrandPrix/_templates

# Run the customize script so people can customize their shell, if they desire
COPY files/* /tmp/
RUN [ -f /tmp/customize.sh ] && $USERSHELL /tmp/customize.sh || $USERSHELL /tmp/customize.sh.template

WORKDIR /root/

ENV USERSHELLPATH=$USERSHELLPATH
CMD $USERSHELLPATH

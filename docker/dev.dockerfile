FROM ros:foxy

LABEL maintainer="Wisconsin Autonomous <wisconsinautonomous@studentorg.wisc.edu"

ARG ROSDISTRO=foxy
ARG DEBIAN_FRONTEND=noninteractive

# Use mirrors instead of main server
RUN sed -i 's|deb http://.*ubuntu.com.* \(focal.*\)|deb mirror://mirrors.ubuntu.com/mirrors.txt \1|g' /etc/apt/sources.list

# Check for updates
RUN apt update && apt upgrade -y

# Install some packages
RUN apt install -y tmux vim ssh git git-lfs zsh python3-pip gxmessage nodejs npm

# Install needed ros packages
COPY workspace/src /tmp/workspace/src/
RUN cd /tmp/workspace && rosdep install --from-paths src --ignore-src -r -y
RUN cd /tmp/ && rm -rf workspace

# Install hygen to create boilerplate code
RUN npm install -g hygen

# Install some python packages
RUN pip install numpy==1.19 pandas matplotlib python-can autobahn tornado twisted Pillow simplejpeg psutil opencv-python wa_simulator

# Some weird stuff for bison
RUN pip install bson && pip install hyperopt && pip install hyperas && pip uninstall bson -y && pip install pymongo

# Various arguments and user settings
ARG USERSHELL
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="/root/.${USERSHELL}rc"

# ROS Setup
RUN sed -i 's|source|#source|g' /ros_entrypoint.sh
RUN echo ". /opt/ros/$ROSDISTRO/setup.sh" >> $USERSHELLPROFILE
RUN echo "[ -f /root/waGrandPrix/workspace/install/setup.$USERSHELL ] && . /root/waGrandPrix/workspace/install/setup.$USERSHELL" >> $USERSHELLPROFILE
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash"

# Environment
ENV HYGEN_TMPLS=/root/waGrandPrix/_templates

# Run the customize script so people can customize their shell, if they desire
COPY docker/files/* /tmp/
RUN [ -f /tmp/customize.sh ] && $USERSHELL /tmp/customize.sh || $USERSHELL /tmp/customize.sh.template

WORKDIR /root/

ENV USERSHELLPATH=$USERSHELLPATH
CMD $USERSHELLPATH

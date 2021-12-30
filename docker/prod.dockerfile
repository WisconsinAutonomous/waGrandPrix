ARG ROSDISTRO=foxy

FROM ros:${ROSDISTRO}

LABEL maintainer="Wisconsin Autonomous <wisconsinautonomous@studentorg.wisc.edu"

ARG ROSDISTRO=foxy
ARG DEBIAN_FRONTEND=noninteractive

# Use mirrors instead of main server
RUN sed -i 's|deb http://.*ubuntu.com.* \(focal.*\)|deb mirror://mirrors.ubuntu.com/mirrors.txt \1|g' /etc/apt/sources.list

# Check for updates
RUN apt update && apt upgrade -y

# Install some packages
RUN apt install -y wget bash python3-pip

# Install some python packages
RUN pip install numpy pandas matplotlib

# Install needed ros packages
COPY workspace/src /tmp/workspace/src/
RUN cd /tmp/workspace && rosdep install --from-paths src --ignore-src -r -y
RUN cd /tmp/ && rm -rf workspace

# Various arguments and user settings
ARG USERSHELL=bash
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="/root/.${USERSHELL}rc"

# ROS Setup
RUN sed -i 's|source|#source|g' /ros_entrypoint.sh
RUN echo ". /opt/ros/$ROSDISTRO/setup.sh" >> $USERSHELLPROFILE
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash"

# Run the customize script so people can customize their shell, if they desire
COPY files/* /tmp/
RUN [ -f /tmp/customize.sh ] && $USERSHELL /tmp/customize.sh || $USERSHELL /tmp/customize.sh.template

WORKDIR /root/

ENV USERSHELLPATH=$USERSHELLPATH
CMD $USERSHELLPATH

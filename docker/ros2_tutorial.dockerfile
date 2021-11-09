FROM ros:foxy

LABEL maintainer="Aaron Young <aryoung5@wisc.edu"

ARG DEBIAN_FRONTEND=noninteractive
ARG ROSDISTRO=foxy

# Use mirrors instead of main server
RUN sed -i 's|deb http://.*ubuntu.com.* \(focal.*\)|deb mirror://mirrors.ubuntu.com/mirrors.txt \1|g' /etc/apt/sources.list

# Check for updates
RUN apt update && apt upgrade -y && apt install -y expect

# Install some packages
RUN apt install -y tmux vim ssh git git-lfs zsh ros-$ROSDISTRO-rviz-common ros-$ROSDISTRO-rviz2
RUN apt install -y python3-pip gxmessage

# Install some python packages
RUN pip install numpy pandas matplotlib

# Various arguments and user settings
ARG USERSHELL
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

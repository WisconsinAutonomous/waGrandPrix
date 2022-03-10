FROM ubuntu:20.04

LABEL maintainer="Wisconsin Autonomous <wisconsinautonomous@studentorg.wisc.edu"

ARG DEBIAN_FRONTEND=noninteractive

# Various arguments and user settings
ARG USERNAME
ARG USERHOME="/home/$USERNAME"
ARG USERSHELL=bash
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="$USERHOME/.${USERSHELL}rc"

# Check for updates
RUN apt update && apt upgrade -y && apt install sudo

# Add user and grant sudo permission.
ARG USER_UID
ARG USER_GID
RUN adduser --shell $USERSHELLPATH --disabled-password --gecos "" \
						--uid $USER_UID --gid $USER_GID $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies
ARG APT_DEPENDENCIES
RUN apt-get update && apt-get install --no-install-recommends -y $APT_DEPENDENCIES

# Clean up to reduce image size
RUN apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# Change to the main user
USER $USERNAME

# Install miniconda
RUN sudo chown -R $USER_UID:$USER_GID /opt
ENV CONDA_DIR /opt/conda
RUN wget --quiet --no-check-certificate https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    	/bin/bash /tmp/miniconda.sh -b -p /opt/conda && \
			rm -rf /tmp/miniconda.sh	

# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH

# Install chrono's packages
ARG PIP_DEPENDENCIES
ARG CONDA_DEPENDENCIES
ARG CONDA_CHANNELS
RUN if [ -n "$CONDA_DEPENDENCIES" ]; then \
			for c in $CONDA_CHANNELS; do \
				conda config --append channels $c;	\
			done; \
			unset CONDA_CHANNELS; \
			conda install $CONDA_DEPENDENCIES; \
    fi
RUN if [ -n "$PIP_DEPENDENCIES" ]; then \
      pip install $PIP_DEPENDENCIES; \
    fi

# Clean up conda
RUN conda clean -a -y

# Default bash config
RUN if [ "$USERSHELL" = "bash" ]; then \
			echo 'export TERM=xterm-256color' >> $USERSHELLPROFILE; \ 
			echo 'export PS1="\[\033[38;5;40m\]\h\[$(tput sgr0)\]:\[$(tput sgr0)\]\[\033[38;5;39m\]\w\[$(tput sgr0)\]\\$ \[$(tput sgr0)\]"' >> $USERSHELLPROFILE; \
		fi

# Set user and work directory
WORKDIR $USERHOME
ENV HOME=$USERHOME
ENV USERSHELLPATH=$USERSHELLPATH

CMD $USERSHELLPATH

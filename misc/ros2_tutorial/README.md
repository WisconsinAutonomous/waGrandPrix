# ROS2 Tutorial

The ROS2 docker container is meant to help people get quickly setup with the [ROS2 tutorials](https://docs.ros.org/en/foxy/Tutorials.html).

You _do not_ need to go through the tutorial titled [Configuring your ROS 2 environment](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html). The docker container has been set up so that this step can be skipped.

Windows users will need to follow this tutorial using a linux environment set up with [WSL](https://docs.microsoft.com/en-us/windows/wsl/about) 

## Running the Docker Container

Make sure you are in the waGrandPrix repository!

### Building

```bash
docker-compose build ros2-tutorial
```

### Starting

```bash
docker-compose up -d ros2-tutorial
```

### Entering

You can run this command in as many terminal windows/tmux panes as you'd like. Replace `<shell>` with whatever shell you prefer; for example, you could use `/bin/zsh` or `/bin/bash` (untested on Windows).

```bash
docker-compose exec ros2-tutorial <shell>
```

### GUI

Further, there may be some issues related to GUI applications. For the ROS 2 tutorials, many utilize the turtlesim application and may not function correctly in docker. This is because docker is meant to be isolated from your normal host system, and lacks any clean functionality to display a GUI window.

There are currently a few work arounds, **but they have only been tested on Mac and Arch Linux systems**.

#### Mac/Linux

Before launching the docker container, you will need to tell X11 (the mechanism on Unix systems that displays windows when forwarding GUIs) to allow applications to display to it.

**WARNING**: There may be some security risks associated with the following command. Use at your own risk (there really shouldn't be much of an issue).

To allow docker to display to X11, run the following command:

```bash
xhost +
```

NOTE: On Mac, if you get `command not found: xhost`, you may need to install xQuartz. To do so, [please see this page](https://macappstore.org/xquartz/).

#### Windows

Currently untested...

### Other Notes

The contents of this folder, outside of this file, are ignored by git.

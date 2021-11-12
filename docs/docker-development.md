# Docker Development

**DISCLAIMER**: For developing the ROS 2 workspace, you _must_ use the created docker images. This is a requirement because we don't want to have to support multiple operating systems and have to deal with individual people's systems. The only place docker will not be used with this repository is on the actual vehicle (and with our own hardware).

Docker is a powerful application that is out of the scope of this README. Beyond Wisconsin Autonomous, knowing what and how to use Docker is a very valuable skill. To learn more, visit [their website](https://www.docker.com/). You can also find a huge amount of tutorials and resources just through Google. Please spend some time understanding Docker before continuing. For the remainder of this tutorial, it is assumed you under stand `docker` and `docker-compose`.

To get started, you will need to install [Docker](https://docs.docker.com/desktop/) and [Docker Compose](https://docs.docker.com/compose/install/).

From anywhere within the `waGrandPrix` repository, do the following:

## 1. Run the container

```
$ docker-compose up -d wagrandprix-dev
```

If this is your first time running this command, the image will need to be built. This may take a few minutes. Unless you are changing things in the dockerfile, this should only have to be done once.

Once it completes, it should say:

```
Creating network "wagrandprix_default" with the default driver
Creating wagrandprix_wagrandprix-dev_1 ... done
```

The container is now running in the background.

## 2. Enter the container

When you enter the container, you may use any shell you'd like. In the following command, be sure to replace `<shell>` with your prefered shell, such as `zsh` or `bash`.

```
docker-compose exec wagrandprix-dev <shell>
```

Within the container, you should enter the shell in `/root/`. Within `/root/`, the only folder you should see is `waGrandPrix`. This is a volume (look up docker volume if you don't know what this is), so any changes you make in this folder will be reflected on your system. Further, the volume is of the existing repository that `docker-compose` was run in.

Various tools are installed to aid development, such as `tmux`. Feel free to leverage these. The `docker-compose exec ...` command can also be run from any terminal window (as long as it's run from within this repository); this is the advantage of running `docker-compose up` in the background (detached).

## 3. Stop the container

When you are finished and would like to free up resources, you may shutdown the image.

```
$ docker-compose down
```

## Additional Notes

For details on how the container was built and what volumes are mounted, refer to the `docker-compose.yml` file and the Dockerfiles in `docker/`.



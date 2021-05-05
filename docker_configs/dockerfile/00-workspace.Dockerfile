FROM ubuntu:20.04
# FROM osrf/ros:melodic-desktop-bionic

ARG APP_UID=1001

ENV \
  DEBIAN_FRONTEND=noninteractive \
  APP_USER=app                   \
  APP_UID=${APP_UID}             \
  DOCKER_GID=999                 \
  USERPASS=1

# Creating a user and installing sudo
RUN \
  useradd -ms /bin/bash -u ${APP_UID} ${APP_USER} && \
  echo ${APP_USER}:${USERPASS} | chpasswd && \
  usermod -aG sudo ${APP_USER} && \
  echo ${APP_USER} 'ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
  chown -R ${APP_USER}:${APP_USER} /home/${APP_USER} && \
  addgroup --gid ${DOCKER_GID} docker && \
  addgroup ${APP_USER} docker && \
  apt-get update && \
  apt-get install -q -y sudo apt-utils alsa-utils --option=Dpkg::Options::=--force-confdef && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /home/${APP_USER}

USER root

# Install packages for fruitful work
RUN apt-get update && apt-get install -q -y \ 
    # apt-utils\
    # alsa-utils \
    iputils-ping \
    liburdfdom-tools \
    mc \
    mesa-utils \
    nano \
    python3-pip \
    tmux \
    htop \
    git \
    software-properties-common \
    psmisc \
    jstest-gtk \
    tree && \
  pip3 install \
    numpy \
    xmlschema \
    pyyaml && \
  rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -q -y openssh-server && \
  rm -rf /var/lib/apt/lists/* && \
  mkdir /run/sshd && \
  sed -i 's/^\#*\(X11Forwarding \).*/\1yes/' /etc/ssh/sshd_config && \
  sed -i 's/^\#*\(X11UseLocalhost \).*/\1no/' /etc/ssh/sshd_config

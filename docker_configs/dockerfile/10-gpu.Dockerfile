USER root

ENV \
  TERM=xterm                                         \
  QT_X11_NO_MITSHM=1                                 \
  NVIDIA_VISIBLE_DEVICES=all                         \
  NVIDIA_DRIVER_CAPABILITIES=compute,utility,display

# Drivers for Nvidia
RUN apt-get update && \
  apt-get install -y --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 && \
  rm -rf /var/lib/apt/lists/*

# Drivers for Intel
RUN apt-get update && \
  apt-get -y install \
    libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

# Neccessary to make work a stack Intel/Nvidia + Gazebo
COPY docker_configs/glvnd_conf.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json
COPY docker_configs/asound.conf /etc/asound.conf
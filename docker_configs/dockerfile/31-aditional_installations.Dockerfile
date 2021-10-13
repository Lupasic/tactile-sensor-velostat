USER root

# Install packages for fruitful work
RUN apt-get update && apt-get install -q -y \ 
    libxtst6 \
    qt5-default  && \
  pip3 install \
    matplotlib \
    urx \
    ur_rtde \
    scipy \
    accessify \
    pyserial && \
  rm -rf /var/lib/apt/lists/*

COPY  docker_configs/extras/urx/* /usr/local/lib/python3.8/dist-packages/urx/

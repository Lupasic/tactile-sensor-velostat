USER root

# Install packages for fruitful work
RUN apt-get update && apt-get install -q -y \ 
    libxtst6 && \
  pip3 install \
    matplotlib \
    pyserial && \
  rm -rf /var/lib/apt/lists/*
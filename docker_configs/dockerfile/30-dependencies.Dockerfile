# Sure that rosdep is installed and initialized
USER ${APP_USER}

COPY . /tmp/dependencies

# Install python2 requirements
# RUN find /tmp/dependencies -name python2requirements.txt -exec pip2 install -r {} \;

# Install python3 requirements
RUN find /tmp/dependencies -name python3requirements.txt -exec pip3 install -r {} \;

RUN sudo rm -rf /tmp/dependencies

USER root

# Add dependency install script
# COPY docker_configs/install_dependencies.sh /home/${APP_USER}/catkin_ws/install_dependencies.sh
# RUN chown ${APP_USER}:${APP_USER} /home/${APP_USER}/catkin_ws/install_dependencies.sh
# RUN chmod 774 /home/${APP_USER}/catkin_ws/install_dependencies.sh

# [PLACE FOR ADDITIONAL DEPENDENCIES]
# RUN \

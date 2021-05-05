USER ${APP_USER}

RUN mkdir /home/app/tactile_sensor_velostat

WORKDIR /home/app/tactile_sensor_velostat

CMD sudo -n /usr/sbin/sshd -D
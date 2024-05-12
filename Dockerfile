FROM ros:noetic-perception-focal

RUN \
    apt update && \
    apt install -y openssh-server sudo xterm
RUN sed -i 's/#*X11Forwarding.*/X11Forwarding yes/' /etc/ssh/sshd_config
RUN sed -i 's/#*X11UseLocalhost.*/X11UseLocalhost no/' /etc/ssh/sshd_config
RUN useradd -m user -p $(openssl passwd 1234)

ENV DISPLAY=host.docker.internal:0.0

EXPOSE 22

CMD service ssh start ; bash
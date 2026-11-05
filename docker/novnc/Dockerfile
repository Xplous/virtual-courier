FROM docker:cli AS docker-cli

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    openbox \
    xterm \
    ca-certificates \
    curl \
    net-tools \
    procps \
    openssh-server \
    sudo \
    xdotool \
 && rm -rf /var/lib/apt/lists/*

COPY --from=docker-cli /usr/local/bin/docker /usr/local/bin/docker

COPY docker/novnc/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 8080 5900 22

ENTRYPOINT ["/entrypoint.sh"]

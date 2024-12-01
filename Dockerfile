FROM openjdk:11

RUN apt-get update

RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends fluxbox lxterminal x11vnc xvfb
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends moreutils
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends chromium
RUN set -ex; \
  useradd nonroot --shell=/bin/bash --create-home --uid=1001; \
  DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends gosu

RUN rm /usr/share/images/fluxbox/debian-squared.jpg  # "fbsetbg: I can't find an app to set the wallpaper with."

RUN mkdir -p /coordination_oru/gradle/wrapper/
WORKDIR /coordination_oru/
COPY gradle* *gradle ./
COPY gradle/wrapper/* gradle/wrapper/
RUN ./gradlew wrapper
VOLUME .gradle/
VOLUME /root/.gradle/

RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends iproute2
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends iputils-ping

ENV IS_CONTAINER=1

CMD ["bash"]

EXPOSE 5900
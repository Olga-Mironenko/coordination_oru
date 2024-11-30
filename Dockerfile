FROM openjdk:11

RUN apt-get update

RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends fluxbox lxterminal x11vnc xvfb
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends moreutils
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends chromium
RUN set -ex; \
  useradd nonroot --shell=/bin/bash --create-home --uid=1001; \
  DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends gosu

RUN rm /usr/share/images/fluxbox/debian-squared.jpg  # "fbsetbg: I can't find an app to set the wallpaper with."

EXPOSE 5900
FROM openjdk:11

RUN apt-get update

RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends fluxbox lxterminal x11vnc xvfb
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends moreutils
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends chromium
RUN set -ex; \
  useradd nonroot --shell=/bin/bash --create-home --uid=1001; \
  DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends gosu

RUN rm /usr/share/images/fluxbox/debian-squared.jpg  # "fbsetbg: I can't find an app to set the wallpaper with."

RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends \
    libompl-dev git g++ cmake make libboost-all-dev
RUN set -eux; \
    cd /opt; \
    git clone https://github.com/ompl/ompl; \
    cd ompl; \
    git reset --hard 02c1139ede4bfbbf26fedcae735631540a15235d; \
    mkdir -p build/Release; \
    cd build/Release; \
    cmake ../..; \
    make -j 4 install; \
    cd ..; \
    rm -rf ompl
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends util-linux  # for `flock`

RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends iproute2 iputils-ping

WORKDIR /coordination_oru/

CMD ["bash"]

EXPOSE 5900
FROM ubuntu:24.04 as base

ENV TZ=Asia/Yerevan
ENV DEBIAN_FRONTEND=noninteractive

# disable certificate check (for kitware)
RUN touch /etc/apt/apt.conf.d/99verify-peer.conf && \
    echo >> /etc/apt/apt.conf.d/99verify-peer.conf "Acquire { https::Verify-Peer false }"

# add kitware repo so that fresh cmake could be installed
RUN apt -y update
RUN apt install -y software-properties-common gpg wget flex bison pkg-config
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && \
    apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
RUN apt -y update
RUN apt install --reinstall ca-certificates
RUN apt install kitware-archive-keyring
RUN apt -y purge --auto-remove cmake && apt -y install cmake

# install essential tools
RUN apt -y update && \
    apt -y install clang make cmake python3-pip git

# Conan
RUN rm /usr/lib/python3.12/EXTERNALLY-MANAGED
RUN pip3 install --force-reinstall --ignore-installed conan

WORKDIR /workdir


# dev
FROM base as dev
COPY . .
ENTRYPOINT bash
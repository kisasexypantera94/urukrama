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
    apt -y install clang make cmake python3-pip git && \
    apt -y remove gcc

RUN apt install -y libopenblas-dev libomp-dev
RUN wget https://github.com/facebookresearch/faiss/archive/refs/tags/v1.8.0.tar.gz && \
    tar xf v1.8.0.tar.gz && \
    cd faiss-1.8.0 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=0 -DFAISS_OPT_LEVEL=generic -DFAISS_ENABLE_PYTHON=OFF -DFAISS_ENABLE_GPU=OFF .. && \
    make -j`$(nproc)` faiss && make install

# Conan
RUN rm /usr/lib/python3.12/EXTERNALLY-MANAGED && \
    pip3 install --force-reinstall --ignore-installed conan && \
    conan profile detect && \
    sed -i 's/compiler.cppstd=gnu17/compiler.cppstd=23/g' ~/.conan2/profiles/default


WORKDIR /workdir


# dev
FROM base as dev
COPY . .
ENTRYPOINT [ "/usr/bin/bash", "-l", "-c" ]
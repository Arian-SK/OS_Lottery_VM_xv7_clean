FROM ubuntu:18.04

RUN apt-get -qq update && \
    apt-get install -yq tzdata && \
    ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

RUN apt-get install -y \
                    build-essential \
                    gdb \
                    tmux \
                    gcc-multilib \
                    qemu-system-x86 \
                    gawk \
                    expect

ADD ./src /xv7-clean

WORKDIR /xv7-clean

ENV TOOLPREFIX=x86_64-linux-gnu-

CMD ["/bin/bash"]

FROM ubuntu:23.10

RUN apt-get update -qq
RUN apt-get upgrade -qqy
RUN apt-get install -qqy --no-install-recommends \
    build-essential \
    ccache \
    clang-17 \
    clang-tidy-17 \
    clang-format-17 \
    g++-13 \
    gcc-13 \
    git \
    lcov \
    libc++-17-dev \
    libc++abi-17-dev \
    libclang-rt-17-dev \
    llvm-17 \
    python3-pip

COPY requirements.txt /requirements.txt
RUN pip3 install -r /requirements.txt --break-system-packages

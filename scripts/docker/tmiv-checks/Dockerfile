FROM ubuntu:22.04

RUN apt-get update -qq \
  && apt-get upgrade -qqy \
  && apt-get install -qqy --no-install-recommends \
    build-essential \
    ccache \
    clang-14 \
    clang-tidy-14 \
    clang-format-14 \
    cmake \
    g++-12 \
    gcc-12 \
    git \
    lcov \
    libc++-14-dev \
    libc++abi-14-dev \
    llvm-14 \
    ninja-build \
    python3-pip

RUN python3 -m pip install gcovr black pytest pyfakefs ninja

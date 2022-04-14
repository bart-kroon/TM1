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

COPY tmiv_integration_test /tmiv_integration_test

RUN python3 -m pip install gcovr black

RUN /usr/sbin/update-ccache-symlinks
ENV PATH=/usr/lib/ccache:$PATH
ENV CCACHE_DIR=/cache/.ccache

RUN mkdir -p /dependencies
RUN git clone --depth 1 --branch v2.13.8 https://github.com/catchorg/Catch2.git /dependencies/Catch2-2_13_8
RUN git clone --depth 1 --branch 8.1.1 https://github.com/fmtlib/fmt.git /dependencies/fmt-8_1_1
RUN git clone --depth 1 --branch HM-16.16 https://vcgit.hhi.fraunhofer.de/jct-vc/HM.git /dependencies/HM-16_16
RUN git clone --depth 1 --branch v1.0.1 https://github.com/fraunhoferhhi/vvdec /dependencies/vvdec-1_0_1
RUN git clone --depth 1 --branch v0.3.1.0 https://github.com/fraunhoferhhi/vvenc /dependencies/vvenc-0_3_1_0

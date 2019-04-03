#!/bin/bash
# Example script for building TMIV with gcc

mkdir -p build.gcc
cd build.gcc
export CC=gcc
export CXX=g++
export CXXFLAGS="-Wall -Wextra -Wpedantic"
cmake \
    -DCMAKE_INSTALL_PREFIX=../install.gcc \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -G "Unix Makefiles" ..
cd ..

make -j 8 -C build.gcc
make -j 8 -C build.gcc test
make -j 8 -C build.gcc install

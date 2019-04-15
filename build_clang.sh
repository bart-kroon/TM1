#!/bin/bash
# Example script for building TMIV with clang

trap exit ERR

mkdir -p build.clang
cd build.clang
export CC=clang
export CXX=clang++
export CXXFLAGS="-Wall -Wextra -Wpedantic -stdlib=libc++"
cmake \
    -DCMAKE_INSTALL_PREFIX=../install.clang \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -G "Unix Makefiles" ..
cd ..

make -j 8 -C build.clang
make -j 8 -C build.clang test
make -j 8 -C build.clang install
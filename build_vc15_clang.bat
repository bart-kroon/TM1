@echo off
rem Example script for generating a Visual Studio project for TMIV with LLVM toolchain
rem
rem 1. Download pre-built binaries Windows (64-bit) from http://releases.llvm.org/download.html
rem 2. Install the LLVM toolchain extension: Tools -> Extensions and Updates -> Online -> Search -> "LLVM"
rem 3. Run this script
rem
rem Tested with TMIV v1.0, LLVM 7.0.1 and Visual Studio 2017 version 15.9.12

mkdir build.clang
cd build.clang
set CXXFLAGS=/W4
cmake -G "Visual Studio 15 2017 Win64" -T LLVM ..
cd ..
cmake --open build.clang

@echo off
rem Example script for generating a Visual Studio project for TMIV

mkdir build
cd build
set CXXFLAGS=/W4
cmake -G "Visual Studio 15 2017 Win64" ..
cd ..
cmake --open build 

cmake_minimum_required(VERSION 3.7)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/catch.hpp")
    add_library(catch "catch.cpp")
	add_library(TMIV::catch ALIAS catch)
    target_compile_features(catch PUBLIC cxx_std_17)
    target_include_directories(catch PUBLIC ".")
endif()

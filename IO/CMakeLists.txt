cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

add_library(IOLib
    "src/IO.cpp"
    "include/TMIV/IO/IO.h"
    "include/TMIV/IO/IO.hpp"
    )

add_library(TMIV::IOLib
    ALIAS IOLib)

target_link_libraries(IOLib
    PUBLIC
        TMIV::DecoderLib)

target_include_directories(IOLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        IOLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

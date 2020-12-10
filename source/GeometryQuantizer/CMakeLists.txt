cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

add_library(GeometryQuantizerLib
    "src/GeometryQuantizer.cpp"
    "src/ExplicitOccupancy.cpp"
    "include/TMIV/GeometryQuantizer/ExplicitOccupancy.h"
    "include/TMIV/GeometryQuantizer/IGeometryQuantizer.h"
    "include/TMIV/GeometryQuantizer/GeometryQuantizer.h"
    )

add_library(TMIV::GeometryQuantizerLib
    ALIAS GeometryQuantizerLib)

target_link_libraries(GeometryQuantizerLib
    PUBLIC
        TMIV::MivBitstreamLib)

target_include_directories(GeometryQuantizerLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        GeometryQuantizerLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

create_catch2_unit_test(
    TARGET
        GeometryQuantizerTest
    SOURCES
        "src/GeometryQuantizer.test.cpp"
    LIBS
        TMIV::GeometryQuantizerLib)

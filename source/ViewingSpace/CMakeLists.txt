cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

add_library(ViewingSpaceLib
    "src/SignedDistance.cpp"
    "src/ViewingSpaceEvaluator.cpp"
    "include/TMIV/ViewingSpace/SignedDistance.h"
    "include/TMIV/ViewingSpace/ViewingSpaceEvaluator.h"
    )

add_library(TMIV::ViewingSpaceLib
    ALIAS ViewingSpaceLib)

target_link_libraries(ViewingSpaceLib
    PUBLIC
        TMIV::MivBitstreamLib)

target_include_directories(ViewingSpaceLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        ViewingSpaceLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

create_catch2_unit_test(
    TARGET
        ViewingSpaceTest
    SOURCES
        "src/ViewingSpace.test.cpp"
    LIBS
        TMIV::ViewingSpaceLib)

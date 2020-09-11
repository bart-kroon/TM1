cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

add_library(AggregatorLib
    "src/Aggregator.cpp"
    "include/TMIV/Aggregator/IAggregator.h"
    "include/TMIV/Aggregator/Aggregator.h"
    )

add_library(TMIV::AggregatorLib
    ALIAS AggregatorLib)

target_link_libraries(AggregatorLib
    PUBLIC
        TMIV::CommonLib)

target_include_directories(AggregatorLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        AggregatorLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

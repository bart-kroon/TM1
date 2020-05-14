cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(ViewOptimizerLib
    "src/ViewReducer.cpp"
    "include/TMIV/ViewOptimizer/IViewOptimizer.h"
    "include/TMIV/ViewOptimizer/ViewReducer.h"
    )

add_library(TMIV::ViewOptimizerLib
    ALIAS ViewOptimizerLib)

target_link_libraries(ViewOptimizerLib
    PUBLIC
        TMIV::RendererLib)

target_include_directories(ViewOptimizerLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        ViewOptimizerLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

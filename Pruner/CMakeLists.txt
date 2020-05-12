cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(PrunerLib
    "src/HierarchicalPruner.cpp"
    "src/PrunedMesh.cpp"
    "src/PrunedMesh.h"
    "src/Pruner.reg.hpp"
    "include/TMIV/Pruner/IPruner.h"
    "include/TMIV/Pruner/HierarchicalPruner.h"
    )

add_library(TMIV::PrunerLib
    ALIAS PrunerLib)

target_link_libraries(PrunerLib
    PUBLIC
        TMIV::RendererLib)

target_include_directories(PrunerLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        PrunerLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

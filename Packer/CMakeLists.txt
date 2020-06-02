cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(PackerLib
    "src/Cluster.cpp"
    "src/Cluster.h"
    "src/MaxRectPiP.cpp"
    "src/MaxRectPiP.h"
    "src/Packer.cpp"
    "include/TMIV/Packer/IPacker.h"
    "include/TMIV/Packer/Packer.h"
    )

add_library(TMIV::PackerLib
    ALIAS PackerLib)

target_link_libraries(PackerLib
    PUBLIC
        TMIV::MivBitstreamLib)

target_include_directories(PackerLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        PackerLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        PackerLib
    SOURCES
        "src/Cluster.cpp"
        "src/MaxRectPiP.cpp"
        "src/MaxRectPiP.h"
        "src/Packer.cpp"
        "include/TMIV/Packer/Cluster.h"
        "include/TMIV/Packer/IPacker.h"
        "include/TMIV/Packer/Packer.h"
    PUBLIC
        TMIV::MivBitstreamLib
    )

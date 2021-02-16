cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        PackerLib
    SOURCES
        "src/Cluster.cpp"
        "src/MaxRectPiP.cpp"
        "src/MaxRectPiP.h"
        "src/Packer.cpp"
        "src/Retriever.cpp"
        "include/TMIV/Packer/Cluster.h"
        "include/TMIV/Packer/IPacker.h"
        "include/TMIV/Packer/Packer.h"
        "include/TMIV/Packer/Retriever.h"
    PUBLIC
        TMIV::MivBitstreamLib
    )

create_catch2_unit_test(
    TARGET
        PackerTest
    SOURCES
        "src/Cluster.test.cpp"
        "src/Packer.test.cpp"
        "src/Retriever.test.cpp"
    PRIVATE
        TMIV::PackerLib
)

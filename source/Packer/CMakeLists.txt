cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(
    TARGET
    PackerLib
    SOURCES
    "src/Cluster.cpp"
    "src/MaxRectPiP.cpp"
    "src/Packer.cpp"
    "src/Retriever.cpp"
    PUBLIC
    MivBitstreamLib)

create_catch2_unit_test(
    TARGET
    PackerTest
    SOURCES
    "test/Cluster_test.cpp"
    "test/MaxRectPiP_test.cpp"
    "test/Retriever_test.cpp"
    PRIVATE
    PackerLib)

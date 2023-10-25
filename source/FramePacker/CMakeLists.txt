cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
    FramePackerLib
    SOURCES
    "src/FramePacker.cpp"
    "src/Stage.cpp"
    PUBLIC
    MivBitstreamLib)

create_catch2_unit_test(TARGET FramePackerTest SOURCES "test/FramePacker_test.cpp" PRIVATE
                        FramePackerLib)

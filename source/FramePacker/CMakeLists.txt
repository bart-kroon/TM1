cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(
    TARGET
    FramePackerLib
    SOURCES
    "src/FramePacker.cpp"
    "src/FramePackerStage.cpp"
    PUBLIC
    MivBitstreamLib)

create_catch2_unit_test(TARGET FramePackerTest SOURCES "test/FramePacker_test.cpp" PRIVATE
                        FramePackerLib)

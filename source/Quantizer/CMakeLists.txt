cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
    QuantizerLib
    SOURCES
    "src/GeometryQuantizer.cpp"
    "src/QuantizerStage.cpp"
    PUBLIC
    MivBitstreamLib)

create_catch2_unit_test(TARGET QuantizerTest SOURCES "test/GeometryQuantizer_test.cpp" PRIVATE
                        QuantizerLib)

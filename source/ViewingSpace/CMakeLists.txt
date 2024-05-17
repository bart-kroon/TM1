cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(
    TARGET
    ViewingSpaceLib
    SOURCES
    "src/SignedDistance.cpp"
    "src/ViewingSpaceEvaluator.cpp"
    PUBLIC
    MivBitstreamLib)

create_catch2_unit_test(TARGET ViewingSpaceTest SOURCES "test/ViewingSpace_test.cpp" PRIVATE
                        ViewingSpaceLib)

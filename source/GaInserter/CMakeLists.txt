cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        GaInserterLib
    SOURCES
        "src/GaInserter.cpp"
    PUBLIC
        MivBitstreamLib
        fmt::fmt
    )

create_tmiv_executable(
    TARGET
        GaInserter
    SOURCES
        "src/GaInserter.main.cpp"
    PRIVATE
        GaInserterLib
        fmt::fmt
    )

create_catch2_unit_test(
    TARGET
        GaInserterTest
    SOURCES
        "src/GaInserter.test.cpp"
    PRIVATE
        GaInserterLib
        fmt::fmt
)

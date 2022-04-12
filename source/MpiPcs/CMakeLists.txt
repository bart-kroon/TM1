cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        MpiPcsLib
    SOURCES
        "src/Frame.cpp"
        "src/MpiPcs.cpp"
    PUBLIC
        IOLib
    PRIVATE
        fmt::fmt
    )

create_tmiv_executable(
    TARGET
        MpiPcs
    SOURCES
        "src/MpiPcs.main.cpp"
    PRIVATE
        MpiPcsLib
        fmt::fmt
    )

create_catch2_unit_test(
    TARGET
        MpiPcsTest
    SOURCES
        "src/MpiPcs.test.cpp"
        "src/Frame.test.cpp"
    PRIVATE
        MpiPcsLib
        fmt::fmt
    )

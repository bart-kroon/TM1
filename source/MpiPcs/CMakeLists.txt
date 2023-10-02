cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
    MpiPcsLib
    SOURCES
    "src/Frame.cpp"
    "src/MpiPcs.cpp"
    PUBLIC
    IOLib)

create_tmiv_executable(TARGET MpiPcs SOURCES "app/MpiPcs_main.cpp" PRIVATE MpiPcsLib)

create_catch2_unit_test(
    TARGET
    MpiPcsTest
    SOURCES
    "test/MpiPcs_test.cpp"
    "test/Frame_test.cpp"
    PRIVATE
    MpiPcsLib)

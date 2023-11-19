cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(TARGET EgaInserterLib SOURCES "src/EgaInserter.cpp" PUBLIC MivBitstreamLib)

create_tmiv_executable(
    TARGET
    EgaInserter
    SOURCES
    "app/EgaInserter_main.cpp"
    PRIVATE
    EgaInserterLib
    ${TMIV_FMT_TARGET})

create_catch2_unit_test(TARGET EgaInserterTest SOURCES "test/EgaInserter_test.cpp" PRIVATE
                        EgaInserterLib)

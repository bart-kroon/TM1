cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        GaInserterLib
    SOURCES
        "src/GaInserter.cpp"
    PUBLIC
        TMIV::MivBitstreamLib
    )

create_tmiv_executable(
    TARGET
        GaInserter
    SOURCES
        "src/GaInserter.main.cpp"
    PRIVATE
        TMIV::GaInserterLib
    )

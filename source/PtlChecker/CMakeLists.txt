cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        PtlCheckerLib
    SOURCES
        "src/PtlChecker.cpp"
    PUBLIC
        TMIV::MivBitstreamLib
    )

create_catch2_unit_test(
    TARGET
        PtlCheckerTest
    SOURCES
        "src/PtlChecker.test.cpp"
    PRIVATE
        TMIV::PtlCheckerLib
    )

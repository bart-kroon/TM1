cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        ParserLib
    SOURCES
        "src/Parser.cpp"
    PUBLIC
        TMIV::MivBitstreamLib
    )

create_tmiv_executable(
    TARGET
        Parser
    SOURCES
        "src/Parser.main.cpp"
    PRIVATE
        TMIV::ParserLib
    )

create_catch2_unit_test(
    TARGET
        ParserTest
    SOURCES
        "src/Parser.test.cpp"
    PRIVATE
        TMIV::ParserLib
)

create_tmiv_executable(
    TARGET
        BitrateReport
    SOURCES
        "src/BitrateReport.main.cpp"
    PRIVATE
        TMIV::MivBitstreamLib
    )

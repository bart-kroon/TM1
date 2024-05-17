cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(
    TARGET
    BitstreamMergerLib
    SOURCES
    "src/BitstreamMerger.cpp"
    PUBLIC
    MivBitstreamLib
    PRIVATE
    DecoderLib
    ${TMIV_FMT_TARGET})

create_tmiv_executable(
    TARGET
    BitstreamMerger
    SOURCES
    "app/BitstreamMerger_main.cpp"
    PRIVATE
    BitstreamMergerLib
    DecoderLib
    IOLib
    ${TMIV_FMT_TARGET})

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        BitstreamMergerLib
    SOURCES
        "src/BitstreamMerger.cpp"
    PUBLIC    
        MivBitstreamLib
    PRIVATE
        DecoderLib
        fmt::fmt
)

create_tmiv_executable(
    TARGET
        BitstreamMerger
    SOURCES
        "src/BitstreamMerger.main.cpp"
    PRIVATE
        BitstreamMergerLib
        DecoderLib
        IOLib
        fmt::fmt
)
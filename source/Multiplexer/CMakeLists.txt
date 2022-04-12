cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        MultiplexerLib
    SOURCES
        "src/DecodeAnnexBStream.cpp"
        "src/EncodeV3cSampleStream.cpp"
        "src/Multiplexer.cpp"
    PUBLIC
        DecoderLib
        VideoDecoderLib
    PRIVATE
        fmt::fmt
)

create_catch2_unit_test(
    TARGET
        MultiplexerTest
    SOURCES
        "src/DecodeAnnexBStream.test.cpp"
        "src/EncodeV3cSampleStream.test.cpp"
        "src/Multiplexer.test.cpp"
    PRIVATE
        MultiplexerLib
)

create_tmiv_executable(
    TARGET
        Multiplexer
    SOURCES
        "src/Multiplexer.main.cpp"
    PRIVATE
        MultiplexerLib
        IOLib
        fmt::fmt
    )

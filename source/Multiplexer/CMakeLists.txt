cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
    MultiplexerLib
    SOURCES
    "src/DecodeAnnexBStream.cpp"
    "src/EncodeV3cSampleStream.cpp"
    "src/Multiplexer.cpp"
    PUBLIC
    MivBitstreamLib
    PRIVATE
    DecoderLib
    VideoDecoderLib)

create_catch2_unit_test(
    TARGET
    MultiplexerTest
    SOURCES
    "test/DecodeAnnexBStream_test.cpp"
    "test/EncodeV3cSampleStream_test.cpp"
    "test/Multiplexer_test.cpp"
    PRIVATE
    MultiplexerLib)

create_tmiv_executable(
    TARGET
    Multiplexer
    SOURCES
    "app/Multiplexer_main.cpp"
    PRIVATE
    MultiplexerLib
    DecoderLib
    VideoDecoderLib
    IOLib)

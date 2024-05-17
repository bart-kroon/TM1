cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(
    TARGET
    DecoderLib
    SOURCES
    "src/DecodeAtlas.cpp"
    "src/DecodeAtlasSubBitstream.cpp"
    "src/DecodeCommonAtlas.cpp"
    "src/DecodeMiv.cpp"
    "src/DecodeNalUnitStream.cpp"
    "src/DecodePatchParamsList.cpp"
    "src/DecodeV3cSampleStream.cpp"
    "src/DecodeVideoSubBitstream.cpp"
    "src/DecodeViewParamsList.cpp"
    "src/GeometryScaler.cpp"
    "src/OutputLog.cpp"
    "src/PreRenderer.cpp"
    "src/V3cUnitBuffer.cpp"
    PUBLIC
    PtlCheckerLib)

create_tmiv_executable(
    TARGET
    Decoder
    SOURCES
    "app/Decoder_main.cpp"
    "app/registerComponents.cpp"
    PRIVATE
    DecoderLib
    VideoDecoderLib
    RendererFrontLib)

create_tmiv_executable(
    TARGET
    DecoderLog
    SOURCES
    "app/DecoderLog_main.cpp"
    PRIVATE
    DecoderLib
    VideoDecoderLib)

create_catch2_unit_test(
    TARGET
    DecoderTest
    SOURCES
    "test/DecodeAtlas_test.cpp"
    "test/DecodeAtlasSubBitstream_test.cpp"
    "test/DecodeCommonAtlas_test.cpp"
    "test/DecodeMiv_test.cpp"
    "test/DecodeNalUnitStream_test.cpp"
    "test/DecodeV3cSampleStream_test.cpp"
    "test/DecodeVideoSubBitstream_test.cpp"
    "test/PreRenderer_test.cpp"
    "test/OutputLog_test.cpp"
    "test/V3cUnitBuffer_test.cpp"
    PRIVATE
    DecoderLib)

create_tmiv_executable(TARGET NativeVideoDecoderTest SOURCES "src/NativeVideoDecoder_test.cpp"
                       PRIVATE DecoderLib)
if(TARGET TmivNativeVideoDecoderTest)
    set_property(TARGET TmivNativeVideoDecoderTest PROPERTY FOLDER "TMIV tests")
endif()

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

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
        PtlCheckerLib
    )

create_tmiv_executable(
    TARGET
        Decoder
    SOURCES
        "src/Decoder.main.cpp"
        "src/registerComponents.cpp"
    PRIVATE
        DecoderLib
        VideoDecoderLib
        RendererFrontLib
    )

create_tmiv_executable(
    TARGET
        DecoderLog
    SOURCES
        "src/DecoderLog.main.cpp"
    PRIVATE
        DecoderLib
        VideoDecoderLib
    )

create_catch2_unit_test(
    TARGET
        DecoderTest
    SOURCES
        "src/DecodeAtlas.test.cpp"
        "src/DecodeAtlasSubBitstream.test.cpp"
        "src/DecodeCommonAtlas.test.cpp"
        "src/DecodeMiv.test.cpp"
        "src/DecodeNalUnitStream.test.cpp"
        "src/DecodeV3cSampleStream.test.cpp"
        "src/DecodeVideoSubBitstream.test.cpp"
        "src/PreRenderer.test.cpp"
        "src/OutputLog.test.cpp"
        "src/V3cUnitBuffer.test.cpp"
    PRIVATE
        DecoderLib
    )

create_tmiv_executable(
    TARGET
        NativeVideoDecoderTest
    SOURCES
        "src/NativeVideoDecoder.test.cpp"
    PRIVATE
        DecoderLib
    )
if (TARGET TmivNativeVideoDecoderTest)
    set_property(TARGET TmivNativeVideoDecoderTest PROPERTY FOLDER "TMIV tests")
endif()

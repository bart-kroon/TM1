cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        DecoderLib
    SOURCES
        "src/AtlasDecoder.cpp"
        "src/CommonAtlasDecoder.cpp"
        "src/EntityBasedPatchMapFilter.cpp"
        "src/FrameUnpacker.cpp"
        "src/GeometryScaler.cpp"
        "src/MivDecoder.cpp"
        "src/OccupancyReconstructor.cpp"
        "src/OutputLog.cpp"
        "src/PreRenderer.cpp"
        "src/V3cSampleStreamDecoder.cpp"
        "src/V3cUnitBuffer.cpp"
    PUBLIC
        TMIV::VideoDecoderLib
    )

create_tmiv_executable(
    TARGET
        Decoder
    SOURCES
        "src/Decoder.main.cpp"
        "src/registerComponents.cpp"
    PRIVATE
        TMIV::DecoderLib
        TMIV::RendererFrontLib
    )

create_catch2_unit_test(
    TARGET
        DecoderTest
    SOURCES
        "src/OutputLog.test.cpp"
        "src/FrameUnpacker.test.cpp"
        "src/CommonAtlasDecoder.test.cpp"
        "src/MivDecoder.test.cpp"
    PRIVATE
        TMIV::DecoderLib
    )

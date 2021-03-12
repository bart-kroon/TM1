cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        DecoderLib
    SOURCES
        "src/AtlasDecoder.cpp"
        "src/CommonAtlasDecoder.cpp"
        "src/Decoder.cpp"
        "src/GeometryScaler.cpp"
        "src/OccupancyReconstructor.cpp"
        "src/EntityBasedPatchMapFilter.cpp"
        "src/MivDecoder.cpp"
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

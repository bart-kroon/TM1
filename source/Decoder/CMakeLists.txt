cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

add_library(DecoderLib
    "src/AtlasDecoder.cpp"
    "src/CommonAtlasDecoder.cpp"
    "src/Decoder.cpp"
    "src/GeometryScaler.cpp"
    "src/OccupancyReconstructor.cpp"
    "src/EntityBasedPatchMapFilter.cpp"
    "src/MivDecoder.cpp"
    "src/NalUnitSemantics.h"
    "src/V3cSampleStreamDecoder.cpp"
    "src/V3cUnitBuffer.cpp"
    "include/TMIV/Decoder/IDecoder.h"
    "include/TMIV/Decoder/AtlasDecoder.h"
    "include/TMIV/Decoder/CommonAtlasDecoder.h"
    "include/TMIV/Decoder/Decoder.h"
    "include/TMIV/Decoder/GeometryScaler.h"
    "include/TMIV/Decoder/OccupancyReconstructor.h"
    "include/TMIV/Decoder/EntityBasedPatchMapFilter.h"
    "include/TMIV/Decoder/MivDecoder.h"
    "include/TMIV/Decoder/V3cSampleStreamDecoder.h"
    "include/TMIV/Decoder/V3cUnitBuffer.h"
    )

add_library(TMIV::DecoderLib
    ALIAS DecoderLib)

target_link_libraries(DecoderLib
    PUBLIC
        TMIV::VideoDecoderLib)

target_include_directories(DecoderLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

add_executable(Decoder
    "src/Decoder.main.cpp"
    "src/registerComponents.cpp"
    )

target_link_libraries(Decoder
    PRIVATE
        TMIV::DecoderLib
        TMIV::RendererFrontLib)

install(
    TARGETS
        Decoder
        DecoderLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

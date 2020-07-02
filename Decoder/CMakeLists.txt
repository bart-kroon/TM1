cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(DecoderLib
    "src/Decoder.cpp"
    "src/GeometryScaler.cpp"
	"src/OccupancyExtractor.cpp"
    "src/EntityBasedPatchMapFilter.cpp"
    "src/MivDecoder.cpp"
    "include/TMIV/Decoder/IDecoder.h"
    "include/TMIV/Decoder/Decoder.h"
    "include/TMIV/Decoder/GeometryScaler.h"
	"include/TMIV/Decoder/OccupancyExtractor.h"
    "include/TMIV/Decoder/EntityBasedPatchMapFilter.h"
    "include/TMIV/Decoder/MivDecoder.h"
    )

add_library(TMIV::DecoderLib
    ALIAS DecoderLib)

target_link_libraries(DecoderLib
    PUBLIC
        TMIV::RendererLib
        TMIV::VideoDecoderLib)

target_include_directories(DecoderLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

add_executable(Decoder
    "src/Decoder.main.cpp"
    "src/registerComponents.cpp"
    "src/IvMetadataReader.cpp"
    "src/IvMetadataReader.h"
    )

target_link_libraries(Decoder
    PRIVATE
        TMIV::DecoderLib
        TMIV::IOLib)

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

if (CMAKE_TESTING_ENABLED)
    add_executable(DecoderTest
        "src/MivDecoder.test.cpp"
        )

    target_link_libraries(DecoderTest
        PRIVATE
            Catch2::Catch2
            TMIV::DecoderLib)

    catch_discover_tests(DecoderTest)

    add_executable(V3cBitstreamTest
        "src/V3cBitstream.test.cpp")

    target_link_libraries(V3cBitstreamTest
        PRIVATE
            Catch2::Catch2
            TMIV::DecoderLib
            TMIV::IOLib)

    catch_discover_tests(V3cBitstreamTest)
endif()

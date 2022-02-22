cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        CommonLib
    SOURCES
        "src/Application.cpp"
        "src/Bitstream.cpp"
        "src/Bytestream.cpp"
        "src/Json.cpp"
        "src/Half.cpp"
        "src/Frame.cpp"
    PUBLIC
        fmt::fmt
    PRIVATE
        Threads::Threads
    )

configure_file(include/TMIV/Common/Version.h.in include/TMIV/Common/Version.h @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/TMIV/Common/Version.h
        DESTINATION include/TMIV/Common)

target_include_directories(CommonLib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        "$<INSTALL_INTERFACE:include>")

target_compile_features(CommonLib
    PUBLIC cxx_std_17)

install(
    TARGETS
        fmt
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

create_catch2_unit_test(
    TARGET
        CommonTest
    SOURCES
        "src/Bitstream.test.cpp"
        "src/Common.test.cpp"
        "src/Decoder.test.cpp"
        "src/Filter.test.cpp"
        "src/FlatMap.test.cpp"
        "src/Frame.test.cpp"
        "src/Graph.test.cpp"
        "src/Json.test.cpp"
        "src/Matrix.test.cpp"
        "src/Quaternion.test.cpp"
        "src/Source.test.cpp"
        "src/verify.test.cpp"
    PRIVATE
        TMIV::CommonLib
    )

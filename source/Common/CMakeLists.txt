cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
    CommonLib
    SOURCES
    "src/Application.cpp"
    "src/Bitstream.cpp"
    "src/Bytestream.cpp"
    "src/Distribution.cpp"
    "src/Json.cpp"
    "src/Half.cpp"
    "src/handleException.cpp"
    "src/Frame.cpp"
    "src/LoggingStrategy.cpp"
    "src/verify.cpp"
    PRIVATE
    Threads::Threads)

if(NOT TARGET CommonLib)
    message(FATAL_ERROR "CommonLib is required in every build of TMIV.")
endif()

configure_file(include/TMIV/Common/Version.h.in include/TMIV/Common/Version.h @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/TMIV/Common/Version.h
        DESTINATION include/TMIV/Common)

target_include_directories(CommonLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
                                            "$<INSTALL_INTERFACE:include>")

target_compile_features(CommonLib PUBLIC cxx_std_17)

create_catch2_unit_test(
    TARGET
    CommonTest
    SOURCES
    "test/Bitstream_test.cpp"
    "test/Common_test.cpp"
    "test/Decoder_test.cpp"
    "test/Distribution_test.cpp"
    "test/Filter_test.cpp"
    "test/FlatMap_test.cpp"
    "test/Frame_test.cpp"
    "test/Graph_test.cpp"
    "test/Json_test.cpp"
    "test/LoggingStrategy_test.cpp"
    "test/LoggingStrategyFmt_test.cpp"
    "test/Matrix_test.cpp"
    "test/Quaternion_test.cpp"
    "test/Source_test.cpp"
    "test/Stage_test.cpp"
    "test/verify_test.cpp"
    PRIVATE
    CommonLib)

cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(CommonLib
    src/Common.cpp
    )

target_link_libraries(CommonLib
    PUBLIC RVS::CommonLib)

target_include_directories(CommonLib
    PUBLIC include)

if (CMAKE_TESTING_ENABLED)
    add_executable(CommonTest
        src/CommonTest.cpp)
    target_link_libraries(CommonTest PRIVATE CommonLib GTest::GTest)
endif()
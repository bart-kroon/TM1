cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

if (BUILD_CATCH2)
	add_library(UnitTestLib "src/UnitTest.main.cpp")
	target_link_libraries(UnitTestLib PUBLIC Catch2::Catch2)
	target_compile_definitions(UnitTestLib PRIVATE CATCH_CONFIG_MAIN)
endif()

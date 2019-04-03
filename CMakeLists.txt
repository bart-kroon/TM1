cmake_minimum_required(VERSION 3.7 FATAL_ERROR)
project(TMIV)

# Read this first: https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1 

# To enable testing, simply download Catch 2 single header version from
# https://raw.githubusercontent.com/catchorg/Catch2/master/single_include/catch2/catch.hpp
# and place it in the catch subdirectory
add_subdirectory(catch)
if (TARGET TMIV::catch)
	enable_testing()
endif()

add_subdirectory(Common)
# add_subdirectory(IO)
# add_subdirectory(Image)

add_subdirectory(Metadata)

add_subdirectory(ViewOptimizer)
add_subdirectory(AtlasConstructor)
# add_subdirectory(AtlasDeconstructor)
add_subdirectory(Renderer)

add_subdirectory(Encoder)
# add_subdirectory(Decoder)

if (CMAKE_TESTING_ENABLED)
	add_executable(Tester "catch/main.cpp")
	target_link_libraries(Tester PRIVATE
		CommonTest
		ViewOptimizerTest
		AtlasConstructorTest
		EncoderTest)
endif()

add_test(NAME Tester COMMAND Tester)

add_custom_target(clang_format COMMAND ${CMAKE_COMMAND} "-P" clang_format.cmake)
add_custom_target(clang_tidy COMMAND "run-clang-tidy.py" "-p" ${CMAKE_CURRENT_BINARY_DIR} "-header-filter" ${CMAKE_CURRENT_SOURCE_DIR} "-fix")

# TODO: Set version of this project

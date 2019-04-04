cmake_minimum_required(VERSION 3.7 FATAL_ERROR)
project(TMIV)

# Read this first: https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1 

# To enable testing, git clone https://github.com/catchorg/Catch2
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/Catch2)
	add_subdirectory(Catch2)
	list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Catch2/contrib)
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
	add_executable(Tester
		"Common/src/Common.test.cpp"
		"ViewOptimizer/src/ViewOptimizer.test.cpp"
		"AtlasConstructor/src/AtlasConstructor.test.cpp"
		"Encoder/src/Encoder.test.cpp"
		"Renderer/src/Renderer.test.cpp")

	target_link_libraries(Tester
		Catch2::Catch2
		TMIV::EncoderLib
		TMIV::RendererLib)

	include(Catch)
	catch_discover_tests(Tester)
endif()

add_custom_target(clang_format COMMAND ${CMAKE_COMMAND} "-DANALYZE_DIR=${CMAKE_SOURCE_DIR}" "-P" ${CMAKE_SOURCE_DIR}/clang_format.cmake )
add_custom_target(clang_tidy COMMAND "run-clang-tidy.py" "-p" ${CMAKE_CURRENT_BINARY_DIR} "-header-filter" ${CMAKE_CURRENT_SOURCE_DIR} "-fix")

# TODO: Set version of this project

cmake_minimum_required(VERSION 3.7 FATAL_ERROR)
project(ITM)

# Read this first: https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1 

find_package(RVS 4 MODULE REQUIRED)

find_package(GTest OPTIONAL)
if (FOUND_GTEST)
	enable_testing()
endif()

add_subdirectory(Common) 				# Common parts shared by most modules, links in RVS::Common
add_subdirectory(IO) 					# Library with streaming operations (but no disc access itself)
add_subdirectory(Image) 				# Image/matrix types and operations, links in RVS::Image
add_subdirectory(ViewReducer) 			# ZJU impl. coordinated by Bin Wang
add_subdirectory(AtlasConstructor) 		# TCH impl., coodrdinated by Julien
add_subdirectory(AtlasDeconstructor) 	# TCH/Philips impl., coordinated by Julien
add_subdirectory(Renderer) 				# Philips/Intel impl., coordinated by Bart

add_subdirectory(Encoder) 				# Entire encoder chain (up to HM), coordinated by Bart
add_subdirectory(Decoder)				# Entire decoder chain (from HM onwards), coordinated by Bart

file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp)
file(GLOB_RECURSE ALL_HEADER_FILES *.hpp)
add_custom_target(CLANG_FORMAT COMMAND clang-format -style=file -i ${ALL_SOURCE_FILES} ${ALL_HEADER_FILES})
add_custom_target(CLANG_TIDY COMMAND run-clang-tidy.py -p ${CMAKE_CURRENT_BINARY_DIR} -header-filter ${CMAKE_CURRENT_SOURCE_DIR} -fix)

cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

file(GLOB_RECURSE ALL_SOURCE_FILES "*.cpp")
file(GLOB_RECURSE ALL_HEADER_FILES "*.h")
file(GLOB_RECURSE ALL_TEMPLATE_FILES "*.hpp")
execute_process(COMMAND "clang-format" "-style=file" "-i" ${ALL_SOURCE_FILES} ${ALL_HEADER_FILES} ${ALL_TEMPLATE_FILES})

add_custom_target(CLANG_TIDY COMMAND "run-clang-tidy.py" "-p" ${CMAKE_CURRENT_BINARY_DIR} "-header-filter" ${CMAKE_CURRENT_SOURCE_DIR} "-fix")

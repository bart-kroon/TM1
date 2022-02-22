cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

file(WRITE "${CMAKE_BINARY_DIR}/CMakeGraphVizOptions.cmake"
    "set(GRAPHVIZ_IGNORE_TARGETS \"Test|Catch2|fmt.header.only\")\n"
    "set(GRAPHVIZ_GENERATE_PER_TARGET FALSE)\n"
    "set(GRAPHVIZ_GENERATE_DEPENDERS FALSE)\n"
)

execute_process(
    COMMAND ${CMAKE_COMMAND} "--graphviz=module_graph" .
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

find_program(DOT_PATH NAMES "dot")
        
if(DOT_PATH)    
    execute_process(
        COMMAND ${DOT_PATH} "module_graph" "-Tsvg" "-o" "${OUTPUT_FILE}"
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    message(STATUS "The module graph has been generated and rendered to ${OUTPUT_FILE}")
else()
    message(WARNING "The module graph has been generated but Graphviz is required to render the .dot-file")
endif()

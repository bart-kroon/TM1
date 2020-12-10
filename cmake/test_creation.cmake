function(create_catch2_unit_test)
    if(NOT CMAKE_TESTING_ENABLED)
        return()
    endif()

    set(prefix TEST_CREATOR)
    set(flags)
    set(singleValues TARGET)
    set(multiValues SOURCES LIBS)

    include(CMakeParseArguments)
    cmake_parse_arguments(${prefix}
            "${flags}"
            "${singleValues}"
            "${multiValues}"
            ${ARGN})

    add_executable(${TEST_CREATOR_TARGET} ${TEST_CREATOR_SOURCES})

    target_link_libraries(${TEST_CREATOR_TARGET}
        PRIVATE
            Catch2::Catch2
            ${TEST_CREATOR_LIBS})

    if(CLANG_TIDY_PATH)
        set_target_properties(${TEST_CREATOR_TARGET}
                PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_PATH};-checks=-readability-function-size,-readability-magic-numbers")
    endif()

    catch_discover_tests(${TEST_CREATOR_TARGET})
endfunction()

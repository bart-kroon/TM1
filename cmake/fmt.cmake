if(MSVC AND "${MSVC_VERSION}" GREATER_EQUAL 1937)
    # Deliberately empty
else()
    find_package(FMT 10 REQUIRED)
    message(STATUS "TMIV: Building on {fmt} ${FMT_VERSION}, found at ${FMT_DIR}.")
    set(TMIV_FMT_TARGET "fmt::fmt")
endif()

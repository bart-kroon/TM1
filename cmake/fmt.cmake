include(CheckCXXSourceCompiles)

check_cxx_source_compiles(
    "
#include <version>

#if 202207L <= __cpp_lib_format && 202207L <= __cpp_lib_print
// OK
#else
#error \"No <format> or <print> support\"
#endif

int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    return 0;
}
"
    HAVE_STD_PRINT)

if(HAVE_STD_PRINT)
    message(STATUS "Using <format> and <print>")
else()
    find_package(FMT REQUIRED)
    message(STATUS "TMIV: Building on {fmt} ${FMT_VERSION}, found at ${FMT_DIR}.")
    set(TMIV_FMT_TARGET "fmt::fmt")
endif()

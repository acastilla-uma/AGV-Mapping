# Google tests
message(STATUS "Build unit tests with Google Test.")
if(NOT (${CMAKE_MAJOR_VERSION} STRLESS 3))
    if(NOT (${CMAKE_MINOR_VERSION} STRLESS 10))
        if(NOT (${CMAKE_PATCH_VERSION} STRLESS 2))
            include(GoogleTest)
        endif()
    endif()
endif()
# The following condition check won't work with cmake 3.5.1
# if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.10.2")
#     include(GoogleTest)
# endif()
add_subdirectory(googletest)

# reference: https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html
mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

# add unit tests
set(UINT_TESTS
    unit_tests/ringbuf_test.cpp
)
add_executable(gtest_all ${UINT_TESTS})
target_link_libraries(gtest_all gtest gmock gtest_main wrp_io)

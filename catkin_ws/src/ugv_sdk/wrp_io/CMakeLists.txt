cmake_minimum_required(VERSION 3.5.1)
project(wrp_io VERSION 0.1.5)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Found ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

## Project Options
option(BUILD_TESTS ON)
option(STATIC_CHECK OFF)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

## Check if pkg is built with ROS catkin
if(CATKIN_DEVEL_PREFIX)
    message(STATUS "Build package with catkin")
    set(BUILD_WITHOUT_ROS OFF)
    find_package(catkin REQUIRED)
else()
    message(STATUS "Build package with cmake")
    set(BUILD_WITHOUT_ROS ON)    
endif()

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(STATIC_CHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        message(STATUS "Found cppcheck")
        set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++11;--enable=all)
    endif() 
endif()

## Additional cmake module path
set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")
list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

## Set compiler to use c++ 11 features
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(BUILD_WITHOUT_ROS)
    ## Use GNUInstallDirs to install libraries into correct locations on all platforms.
    include(GNUInstallDirs)

    ## Put all binary files into /bin and libraries into /lib
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
endif()

# Build libraries
find_package(Threads REQUIRED)

# import asio library
# add_subdirectory(include/asio)
add_library(asio INTERFACE)
target_compile_definitions(asio INTERFACE "-DASIO_STANDALONE")
target_include_directories(asio INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/asio/include>
	$<INSTALL_INTERFACE:include>)

add_library(${PROJECT_NAME} 
    src/async_serial.cpp
    src/async_can.cpp
)
target_link_libraries(${PROJECT_NAME} asio Threads::Threads)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
    PRIVATE src)
    
# Build tests
if(PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME AND BUILD_TESTS)
    message(STATUS "Tests will be built")
    enable_testing()
    add_subdirectory(test)
else()
    message(STATUS "Tests will not be built")
endif()

## Installation configuration
if(BUILD_WITHOUT_ROS) # BUILD_WITHOUT_ROS

    message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX} with 'make install'")

    # Offer the user the choice of overriding the installation directories
    set(INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR} CACHE PATH "Installation directory for libraries")
    set(INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR} CACHE PATH "Installation directory for executables")
    set(INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
    if(WIN32 AND NOT CYGWIN)
        set(DEF_INSTALL_CMAKEDIR CMake)
    else()
        set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
    endif()
    set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")

    # Report to user
    foreach(p LIB BIN INCLUDE CMAKE)
        file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}DIR} _path)
        message(STATUS "Installation path of ${p} components: ${_path}")
        unset(_path)
    endforeach()

    # targets to install
    install(TARGETS ${PROJECT_NAME} asio
        EXPORT wrp_ioTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

    install(DIRECTORY include/wrp_io include/asio/include/asio
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    install(FILES include/asio/include/asio.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    # export target configuration 
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        wrp_ioConfigVersion.cmake
        VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}"
        COMPATIBILITY SameMajorVersion)

    install(EXPORT wrp_ioTargets
        FILE wrp_ioTargets.cmake
        NAMESPACE westonrobot::
        DESTINATION lib/cmake/wrp_io)

    configure_file(cmake/wrp_ioConfig.cmake.in wrp_ioConfig.cmake @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/wrp_ioConfig.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/wrp_ioConfigVersion.cmake"
            DESTINATION lib/cmake/wrp_io)

    # Packaging support
    set(CPACK_PACKAGE_VENDOR "Weston Robot")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Weston Robot Platform - Linux Driver for IO")
    set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
    set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
    set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
    set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/weston_robot")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
    set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

    set(CPACK_GENERATOR "DEB")
    set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Ruixiang Du (ruixiang.du@westonrobot.com)") 
    # set(CPACK_DEBIAN_PACKAGE_DEPENDS "libasio-dev")
    set(CPACK_SOURCE_IGNORE_FILES .git dist .*build.* /\\\\.DS_Store)
    include(CPack)

else() # BUILD_WITHOUT_ROS

    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include include/asio/include
        #  CATKIN_DEPENDS rospy scout_msgs std_msgs
        #  DEPENDS system_lib
    )

    ## Add catkin install targets
    install(TARGETS ${PROJECT_NAME}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

    install(DIRECTORY include/${PROJECT_NAME} include/asio/include/asio
      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

    install(FILES include/asio/include/asio.hpp
      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

endif() # BUILD_WITHOUT_ROS

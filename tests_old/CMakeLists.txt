# Old Test Configuration for Navigation Filter Library
#
# This file defines the build system for legacy unit tests that were developed 
# against previous versions of the navigation filter library. These tests are 
# no longer compatible with the current codebase and are preserved for historical 
# reference only. They should not be used in active development.
#
# NOTE: This configuration is deprecated and not maintained. New tests should
# be developed in the main test framework using updated APIs.
#
# @author peanut-nav
# @date Created: 2025-07-22
# @last Modified: 2025-07-22 (marked as deprecated)
# @version 0.1 (deprecated)
# @deprecated This test configuration is no longer compatible with current code

# Minimum required CMake version
cmake_minimum_required(VERSION 3.15)

# Add test executables for legacy tests
add_executable(test_data_loader test_data_loader.cpp)
add_executable(test_navigation_core test_navigation_core.cpp)  # Legacy navigation core test

# Link libraries for legacy test_data_loader
target_link_libraries(test_data_loader
    PRIVATE
    GTest::gtest_main       # Google Test main function (legacy)
    MathUtils              # Mathematical utilities (legacy version)
    DataLoader             # Data loading functionality (legacy version)
    SystemInitializer      # System initialization (legacy version)
)

# Link libraries for legacy test_navigation_core
target_link_libraries(test_navigation_core
    PRIVATE
    GTest::gtest_main       # Google Test main function (legacy)
    MathUtils              # Mathematical utilities (legacy version)
    DataLoader             # Data loading functionality (legacy version)
    SystemInitializer      # System initialization (legacy version)
    NavigationCore         # Core navigation algorithms (legacy version)
)

# Register legacy tests with CTest (not recommended for current use)
add_test(NAME test_data_loader COMMAND test_data_loader)
add_test(NAME test_navigation_core COMMAND test_navigation_core)  # Legacy test registration

# Filesystem support for different compilers (legacy configuration)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Legacy compiler flags for C++17 and filesystem
    target_compile_options(test_data_loader PRIVATE "-std=c++17")
    target_link_libraries(test_data_loader PRIVATE stdc++fs)
    
    target_compile_options(test_navigation_core PRIVATE "-std=c++17")
    target_link_libraries(test_navigation_core PRIVATE stdc++fs)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # Legacy compiler flags for MSVC
    target_compile_options(test_data_loader PRIVATE "/std:c++17")
    target_compile_options(test_navigation_core PRIVATE "/std:c++17")
endif()

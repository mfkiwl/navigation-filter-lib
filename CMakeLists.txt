# CMake configuration for Navigation Filter Library
#
# This file defines the build system for the integrated navigation filter library,
# including core math utilities, data loading, and navigation algorithms.
#
# @author peanut-nav
# @date Created: 2025-07-22
# @version 0.1

# Minimum required CMake version
cmake_minimum_required(VERSION 3.15)

# Project definition
project(NavigationFilterLib
    VERSION 0.1.0
    DESCRIPTION "Integrated Navigation Filter Library"
    LANGUAGES CXX
)

# Set C++ standard and requirements
set(CMAKE_CXX_STANDARD 17)          # Require C++17 features
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Enforce C++17 standard
set(CMAKE_CXX_EXTENSIONS OFF)        # Disable compiler-specific extensions

# Set output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)   # Static libraries (.a)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)   # Shared libraries (.so)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)   # Executables

# Find required dependencies (only Eigen3, no GTest)
find_package(Eigen3 3.3 REQUIRED)    # Required for matrix and vector operations

# Set project include directories
include_directories(
    include         # Main include directory for project headers
    ${EIGEN3_INCLUDE_DIR}  # Eigen include directory
)

# MathUtils library (interface target for mathematical utilities)
add_library(MathUtils INTERFACE)
target_link_libraries(MathUtils INTERFACE Eigen3::Eigen)
target_include_directories(MathUtils INTERFACE include)

# DataLoader library (handles loading and preprocessing of sensor data)
add_library(DataLoader src/DataLoader.cpp)
target_link_libraries(DataLoader MathUtils)
target_include_directories(DataLoader PUBLIC include)

# SystemInitializer library (initializes navigation parameters and state)
add_library(SystemInitializer 
    src/initializers/KfInitializer.cpp
)
target_link_libraries(SystemInitializer MathUtils DataLoader)
target_include_directories(SystemInitializer 
    PUBLIC 
    include
    include/initializers
)

# NavigationCore library (core navigation algorithms and Kalman filtering)
add_library(NavigationCore src/NavigationCore.cpp)
target_link_libraries(NavigationCore 
    MathUtils 
    DataLoader 
    SystemInitializer
)
target_include_directories(NavigationCore PUBLIC include)

# SaveResults library (handles saving of navigation results)
add_library(SaveResults src/SaveResults.cpp)
target_link_libraries(SaveResults MathUtils)
target_include_directories(SaveResults PUBLIC include)

# Main application executable
add_executable(main_app src/main.cpp)
target_link_libraries(main_app 
    DataLoader 
    SystemInitializer 
    NavigationCore 
    SaveResults
)
# Set output directory for the executable
set_target_properties(main_app PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Print configuration summary
message(STATUS "=============================================")
message(STATUS "Navigation Filter Library Configuration Summary")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Executable Output: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "  Library Output: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "=============================================")

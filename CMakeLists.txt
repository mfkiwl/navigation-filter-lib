# CMake configuration for Navigation Filter Library
#
# This file defines the build system for the integrated navigation filter library,
# including core math utilities, data loading, and navigation algorithms.
#
# @author peanut-nav
# @date Created: 2025-08-04
# @version 0.3.0

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
    include/params  # Navigation parameters headers
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
    src/initializers/EkfInitializer.cpp
)
target_link_libraries(SystemInitializer MathUtils DataLoader)
target_include_directories(SystemInitializer 
    PUBLIC 
    include
    include/initializers
    include/params
)

# NavigationAlgorithms library (core navigation algorithms and filtering)
add_library(NavigationAlgorithms 
    src/core/KfNavigation.cpp
    src/core/EkfNavigation.cpp
    src/core/RtsSmoother.cpp
)
target_link_libraries(NavigationAlgorithms 
    MathUtils 
    DataLoader 
    SystemInitializer
)
target_include_directories(NavigationAlgorithms 
    PUBLIC 
    include
    include/core
    include/params
)

# SaveResults library (handles saving of navigation results)
add_library(SaveResults src/SaveResults.cpp)
target_link_libraries(SaveResults MathUtils)
target_include_directories(SaveResults PUBLIC include)

# KF
add_executable(IntegratedNavigationKF 
    src/main_kf.cpp
)
target_link_libraries(IntegratedNavigationKF 
    DataLoader 
    SystemInitializer 
    NavigationAlgorithms 
    SaveResults
)
set_target_properties(IntegratedNavigationKF PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# EKF
add_executable(IntegratedNavigationEKF 
    src/main_ekf.cpp
)
target_link_libraries(IntegratedNavigationEKF 
    DataLoader 
    SystemInitializer 
    NavigationAlgorithms 
    SaveResults
)
set_target_properties(IntegratedNavigationEKF PROPERTIES
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
message(STATUS "  Libraries: ")
message(STATUS "    - MathUtils")
message(STATUS "    - DataLoader")
message(STATUS "    - SystemInitializer")
message(STATUS "    - NavigationAlgorithms (KF and EKF)")
message(STATUS "    - SaveResults")
message(STATUS "  Executables: ")
message(STATUS "    - IntegratedNavigationKF")
message(STATUS "    - IntegratedNavigationEKF")
message(STATUS "=============================================")

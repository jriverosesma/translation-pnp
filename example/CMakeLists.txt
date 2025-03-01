# Set minimum required version
cmake_minimum_required(VERSION 3.25.0)

# Set project name and version
project(ExampleProject VERSION 0.1 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Eigen is required by TPnP and main.cpp
find_package(Eigen3 3.4.0 REQUIRED)

# OpenCV is required by TPnP
find_package(OpenCV 4.10.0 REQUIRED)

# Find TPnP library
find_package(TPnP REQUIRED)

# Add an executable that uses the TPnP library
add_executable(example_app main.cpp)

# Link TPnP library
target_link_libraries(example_app PRIVATE TPnP::TPnP)

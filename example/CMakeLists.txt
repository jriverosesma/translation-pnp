# Set minimum required version
cmake_minimum_required(VERSION 3.22.0)

# Set project name and version
project(ExampleProject VERSION 0.1 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find MyLib library
find_package(MyLib REQUIRED)

# Add an executable that uses the MyLib library
add_executable(example_app main.cpp)

# Link MyLib library
target_link_libraries(example_app PRIVATE MyLib::MyLib)

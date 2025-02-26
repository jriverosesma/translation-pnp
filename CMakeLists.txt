# Set minimum required version
cmake_minimum_required(VERSION 3.22.0)

# Set project name and version
project(MyLib VERSION 0.1 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add CMake's FetchContent module to download and integrate libraries into the project
include(FetchContent)

# Option to see git clone progress
set(FETCHCONTENT_QUIET FALSE)

# Fetch pybind11 if not found
find_package(pybind11 2.13.5 QUIET)
if(NOT pybind11_FOUND)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.5
  )
  FetchContent_MakeAvailable(pybind11)
endif()

# Fetch Catch2 if not found
find_package(Catch2 3.7.0 QUIET)
if(NOT Catch2_FOUND)
  FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.7.0
    GIT_PROGRESS TRUE
  )
  FetchContent_MakeAvailable(catch2)
endif()

# Find all .cpp files in the src directory except the Python binding
file(GLOB_RECURSE MYLIB_SOURCES "src/mylib.cpp" "src/utils/*.cpp")

# Add the library target with all .cpp files in src
add_library(MyLib STATIC ${MYLIB_SOURCES})

# Specify include directories for the MyLib library target
target_include_directories(MyLib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # During build
  $<INSTALL_INTERFACE:include>                            # After install
)

# pybind11 requires for all libraries to be linked to be compiled with -fPIC option
set_target_properties(MyLib PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ================= PYTHON BINDING CONFIGURATION ==================

# Add the Python module using pybind11
pybind11_add_module(mylib_cpp "src/binding.cpp")

# Link MyLib library and third-party libraries with the Python module
target_link_libraries(mylib_cpp PRIVATE MyLib)

# Install the Python module
install(TARGETS mylib_cpp LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# ================= TESTING CONFIGURATION ==================

# Find all .cpp files in the tests directory and add them to the test executable
file(GLOB_RECURSE TEST_SOURCES "tests/*.cpp")

# Add the test executable
add_executable(tests ${TEST_SOURCES})

# Link MyLib library with the test executable
target_link_libraries(tests PRIVATE MyLib Catch2::Catch2WithMain)

# Enable testing
enable_testing()

# Add test MyLibTests that runs tests as part of the project's test suite
add_test(NAME MyLibTests COMMAND tests)

# Run tests after the test executable is built
add_custom_command(
    TARGET tests
    POST_BUILD
    COMMAND ${CMAKE_CTEST_COMMAND} -R MyLibTests
    COMMENT "Running tests..."
)

# ================= INSTALLATION CONFIGURATION ==================

# Installation rules for MyLib library
install(TARGETS MyLib
    EXPORT MyLibTargets                   # Export target
    ARCHIVE DESTINATION lib               # Static library location
    LIBRARY DESTINATION lib               # Shared library location
    RUNTIME DESTINATION bin               # Binary/executable location
    INCLUDES DESTINATION include          # Include headers
)

# Install the public headers
install(DIRECTORY include/ DESTINATION include)

# Export the targets for other projects
install(EXPORT MyLibTargets
    FILE MyLibTargets.cmake
    NAMESPACE MyLib::
    DESTINATION lib/cmake/MyLib
)

# Package configuration
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/MyLibConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/MyLibConfigVersion.cmake"
    DESTINATION lib/cmake/MyLib
)

# Create a package configuration version file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/MyLibConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Configure file for find_package() support
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/MyLibConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/MyLibConfig.cmake"
    INSTALL_DESTINATION lib/cmake/MyLib
)

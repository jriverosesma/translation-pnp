# Set minimum required version
cmake_minimum_required(VERSION 3.25.0)

# Set project name and version
project(TPnP VERSION 0.1 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add CMake's FetchContent module to download and integrate libraries into the project
include(FetchContent)

# Option to see git clone progress
set(FETCHCONTENT_QUIET FALSE)

# Fetch Eigen if not found
find_package(Eigen3 3.4.0 QUIET)
if(NOT Eigen3_FOUND)
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_PROGRESS TRUE
    CMAKE_ARGS
      -DEIGEN_BUILD_TESTING=OFF
      -DEIGEN_MPL2_ONLY=ON
      -DEIGEN_BUILD_PKGCONFIG=OFF
      -DEIGEN_BUILD_DOC=OFF
  )
  FetchContent_MakeAvailable(eigen)
endif()

# Fetch OpenCV if not found
find_package(OpenCV 4.10.0 QUIET)
if(NOT OpenCV_FOUND)
  FetchContent_Declare(
    opencv
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG 4.10.0
    GIT_PROGRESS TRUE
    CMAKE_ARGS
      -DBUILD_EXAMPLES=OFF
      -DBUILD_TESTS=OFF
      -DBUILD_DOCS=OFF
      -DBUILD_PERF_TESTS=OFF
      -DBUILD_opencv_python_bindings_generator=OFF
      -DBUILD_opencv_java=OFF
      -DWITH_IPP=OFF
      -DWITH_TBB=OFF
      -DWITH_CUDA=OFF
      -DBUILD_SHARED_LIBS=ON
  )
  FetchContent_MakeAvailable(opencv)
endif()

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
file(GLOB_RECURSE TPNP_SOURCES "src/pose.cpp" "src/utils/*.cpp")

# Add the library target with all .cpp files in src
add_library(TPnP STATIC ${TPNP_SOURCES})

# Specify include directories for the TPnP library target
target_include_directories(TPnP PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # During build
  $<INSTALL_INTERFACE:include>                            # After install
  $<BUILD_INTERFACE:${OPENCV_CONFIG_FILE_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${OPENCV_MODULE_opencv_core_LOCATION}/include>
  $<BUILD_INTERFACE:${OPENCV_MODULE_opencv_calib3d_LOCATION}/include>
  $<BUILD_INTERFACE:${OPENCV_MODULE_opencv_features2d_LOCATION}/include>
  $<BUILD_INTERFACE:${OPENCV_MODULE_opencv_flann_LOCATION}/include>
)

# pybind11 requires for all libraries to be linked to be compiled with -fPIC option
set_target_properties(TPnP PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Link third-party libraries with TPnP library
target_link_libraries(TPnP PUBLIC Eigen3::Eigen opencv_core opencv_calib3d opencv_imgproc opencv_features2d opencv_flann)

# ================= PYTHON BINDING CONFIGURATION ==================

# Add the Python module using pybind11
pybind11_add_module(tpnp_cpp "src/binding.cpp")

# Link TPnP library and third-party libraries with the Python module
target_link_libraries(tpnp_cpp PRIVATE TPnP)

# Install the Python module
install(TARGETS tpnp_cpp LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# ================= TESTING CONFIGURATION ==================

# Find all .cpp files in the tests directory and add them to the test executable
file(GLOB_RECURSE TEST_SOURCES "tests/*.cpp")

# Add the test executable
add_executable(tests ${TEST_SOURCES})

# Link Eigen, OpenCV (through TPnP) and the TPnP library with the test executable
target_link_libraries(tests PRIVATE TPnP Catch2::Catch2WithMain)

# Enable testing
enable_testing()

# Add test TPnPTests that runs tests as part of the project's test suite
add_test(NAME TPnPTests COMMAND tests)

# Run tests after the test executable is built
add_custom_command(
    TARGET tests
    POST_BUILD
    COMMAND ${CMAKE_CTEST_COMMAND} -R TPnPTests
    COMMENT "Running tests..."
)

# ================= INSTALLATION CONFIGURATION ==================

# Installation rules for TPnP library
install(TARGETS TPnP
    EXPORT TPnPTargets                    # Export target
    ARCHIVE DESTINATION lib               # Static library location
    LIBRARY DESTINATION lib               # Shared library location
    RUNTIME DESTINATION bin               # Binary/executable location
    INCLUDES DESTINATION include          # Include headers
)

# Install the public headers
install(DIRECTORY include/ DESTINATION include)

# Export the targets for other projects
install(EXPORT TPnPTargets
    FILE TPnPTargets.cmake
    NAMESPACE TPnP::
    DESTINATION lib/cmake/TPnP
)

# Package configuration
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/TPnPConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/TPnPConfigVersion.cmake"
    DESTINATION lib/cmake/TPnP
)

# Create a package configuration version file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/TPnPConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Configure file for find_package() support
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/TPnPConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/TPnPConfig.cmake"
    INSTALL_DESTINATION lib/cmake/TPnP
)

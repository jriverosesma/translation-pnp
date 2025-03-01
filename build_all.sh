#!/usr/bin/env bash

# Default configuration
BUILD_TYPE="release"

# Function to print usage
usage() {
    echo "Usage: $0 [-t build_type]"
    echo "Options:"
    echo "  -t    Set build type (release, debug). Default is release."
    exit 1
}

# Parse command-line arguments
while getopts ":t:" opt; do
    case "${opt}" in
        t)
            BUILD_TYPE=${OPTARG}
            if [[ "$BUILD_TYPE" != "release" && "$BUILD_TYPE" != "debug" ]]; then
                echo "Invalid build type: $BUILD_TYPE"
                usage
            fi
            ;;
        *)
            usage
            ;;
    esac
done

# Navigate to directory where this script is defined
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd $SCRIPT_DIR

# Unset proxy settings to fetch content using CMake's FetchContent module
# unset http_proxy
# unset https_proxy

# Build TPnP library
echo "Building TPnP library with build type: $BUILD_TYPE..."
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -B build -S . && cmake --build build

# Install TPnP library
echo "Installing TPnP library..."
cmake --install build --prefix build/install

# Build example project
echo "Building example project..."
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_PREFIX_PATH=../build/install -B example/build -S example/ && cmake --build example/build

# Make sure shared libraries needed by TPnP can be found by the dynamic link loader during runtime.
# For TPnP this is basically OpenCV. This line is not needed if OpenCV can already be found by the
# dynamic link loader, which is the case if for instance it has already been installed at system level.
# This is needed to run the C++ as well as the Python code.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${SCRIPT_DIR}/build/install/lib

# Run C++ example
echo "Running C++ example..."
./example/build/example_app

# Run Python example
echo "Running Python example..."
python3 example/main.py

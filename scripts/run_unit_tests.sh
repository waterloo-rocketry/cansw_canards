#!/bin/bash
# Exit immediately if any command fails.
set -e

# Determine the project root directory (assumes this script is in a subfolder)
ROOT_DIR=$(dirname "$(dirname "$(realpath "$0" || echo "$(pwd)/$0")")")
BUILD_ROOT="${ROOT_DIR}/build"

# Functions to map a target to the preset names defined in cmakepresets.json
get_build_preset() {
  case "$1" in
    test|tests)
      echo "test"
      ;;
    *)
      echo ""
      ;;
  esac
}

show_help() {
  echo "Usage: $0 {build|test|cleanall|help} [target]"
  echo
  echo "Commands:"
  echo "  build [target]   Build the project for [target]."
  echo "                   Targets: test"
  echo "  test             Build tests and run them."
  echo "  cleanall         Remove the entire build folder (${BUILD_ROOT})."
  echo "  help             Display this help message."
}

# Main script logic
COMMAND=$1
TARGET=$2

# If a target was provided, convert it to lowercase.
if [ -n "$TARGET" ]; then
  TARGET=$(echo "$TARGET" | tr '[:upper:]' '[:lower:]')
fi

case "$COMMAND" in
  build)
    if [ -z "$TARGET" ]; then
      echo "Please specify a target: test"
      exit 1
    fi
    BUILD_PRESET=$(get_build_preset "$TARGET")
    if [ -z "$BUILD_PRESET" ]; then
      echo "Unknown target: $TARGET"
      exit 1
    fi
    echo "Building target '$TARGET' using preset '$BUILD_PRESET'..."
    # This call will configure (if needed) and then build the project.
    cmake --preset "$BUILD_PRESET"
    cmake --build --preset "${BUILD_PRESET}-no-format"
    echo "Build completed."
    ;;
  test)
    # For testing, we use the test preset.
    echo "Building and running tests..."
    BUILD_PRESET=$(get_build_preset test)
    TEST_PRESET="test"
    if [ -z "$BUILD_PRESET" ] || [ -z "$TEST_PRESET" ]; then
      echo "Error: Missing presets for test target."
      exit 1
    fi
    cmake --preset "$BUILD_PRESET"
    # building also runs the tests too via gcov/lcov setup
    cmake --build --preset "$BUILD_PRESET" --parallel
    echo "Tests completed."
    ;;
  cleanall)
    echo "Cleaning entire build directory: ${BUILD_ROOT}"
    rm -rf "${BUILD_ROOT}"
    echo "Entire build directory removed."
    ;;
  help|*)
    show_help
    ;;
esac
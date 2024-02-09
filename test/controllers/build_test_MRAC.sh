#!/bin/bash

# Compiler
CXX=g++

# Compiler flags
CXXFLAGS="-std=c++17 -Wall"

# Path to Eigen library (change this to match your Eigen installation path)
EIGEN_PATH="/Users/priyankabhovad/Downloads/cpp_libs/eigen"

# Include flags for Eigen
INCLUDE_FLAGS="-I$EIGEN_PATH"

# Include local include path
LOCAL_INCLUDE_PATH="/Users/priyankabhovad/Downloads/cpp_ex/system_modeling_and_control/src"

# Include flags for local header files
LOCAL_INCLUDE_FLAGS="-I$LOCAL_INCLUDE_PATH"

SRCS="test/controllers/test_MRAC.cpp
     src/controllers/model_reference_adaptive_controller.cpp
     src/numerical_solvers/rk_ode_solver.cpp
     src/system_models/linear_time_invariant_system.cpp"

# Output executable name
TARGET="build/myprogram"

# Compilation command
$CXX $CXXFLAGS $INCLUDE_FLAGS $LOCAL_INCLUDE_FLAGS -o $TARGET $SRCS

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful. Executable created: $TARGET"
else
    echo "Compilation failed."
fi

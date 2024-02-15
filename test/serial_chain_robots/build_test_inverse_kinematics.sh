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

SRCS="test/serial_chain_robots/test_inverse_kinematics.cpp 
     src/serial_chain_robots/serial_chain_robot_helper_funs.cpp
     src/serial_chain_robots/inverse_kinematics.cpp
     src/serial_chain_robots/forward_kinematics.cpp
     src/numerical_solvers/newton_raphson.cpp"

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

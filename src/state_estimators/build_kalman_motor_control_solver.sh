#!/bin/bash

# Compiler
CXX=g++

# Compiler flags
CXXFLAGS="-std=c++17 -Wall"

# Path to Eigen library (change this to match your Eigen installation path)
EIGEN_PATH="/Users/priyankabhovad/Downloads/cpp_libs/eigen"

# Include flags for Eigen
INCLUDE_FLAGS="-I$EIGEN_PATH"

# Source files
SRCS="pid_plus_motor_control_solver.cpp pid_controller.cpp rk_ode_solver_w_eigen.cpp savecsv.cpp"

# Output executable name
TARGET="myprogram"

# Compilation command
$CXX $CXXFLAGS $INCLUDE_FLAGS -o $TARGET $SRCS

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful. Executable created: $TARGET"
else
    echo "Compilation failed."
fi

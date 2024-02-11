#!/bin/bash

# Compiler
CXX=g++

# Compiler flags
CXXFLAGS="-std=c++17 -Wall"

# Path to Eigen library (change this to match your Eigen installation path)
EIGEN_PATH="/Users/priyankabhovad/Downloads/cpp_libs/eigen"

# Include flags for Eigen
EIGEN_FLAGS="-I$EIGEN_PATH"

# Path to Eigen library (change this to match your Eigen installation path)
NLOPT_PATH="/opt/homebrew/Cellar/nlopt/2.7.1/include"
NLOPT_LIB_PATH="/opt/homebrew/Cellar/nlopt/2.7.1/lib"

# Include flags for Eigen
NLOPT_FLAGS="-I$NLOPT_PATH"

NLOPT_LIB_FLAGS="-L$NLOPT_LIB_PATH"

# Include local include path
LOCAL_INCLUDE_PATH="/Users/priyankabhovad/Downloads/cpp_ex/system_modeling_and_control/src"

# Include flags for local header files
LOCAL_INCLUDE_FLAGS="-I$LOCAL_INCLUDE_PATH"

# Source files
SRCS="examples/nlopt_usage_vec_constraint.cpp 
      src/data_logging/savecsv.cpp
      src/data_logging/data_logging_helper_funs.cpp"

# Output executable name
TARGET="build/myprogram"

# Compilation command
$CXX $CXXFLAGS $EIGEN_FLAGS $NLOPT_FLAGS $NLOPT_LIB_FLAGS $LOCAL_INCLUDE_FLAGS -o $TARGET $SRCS -lnlopt

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful. Executable created: $TARGET"
else
    echo "Compilation failed."
fi

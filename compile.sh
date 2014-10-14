#!/bin/bash

BASEDIR=$(dirname $0)
cd $BASEDIR

mkdir build
cd build
make clean
cmake ../ -DCMAKE_CXX_COMPILER=colorgcc -DCMAKE_BUILD_TYPE=Release
make -j 12

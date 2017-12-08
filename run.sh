#!/bin/bash
# Script to compile and run particle filter!
#
# Written by Tiffany Huang, 12/14/2016
# Updated by Leo Lei, 12/08/2017
#

# Compile and Run particle filter
cd ./build
cmake ..
make
./particle_filter

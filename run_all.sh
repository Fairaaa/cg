#!/usr/bin/env bash

cmake -B build
cmake --build build

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output
# build/PA1 testcases/scene01_basic.txt output/scene01.bmp 50
# build/PA1 testcases/scene02_cube.txt output/scene02.bmp
build/PA1 testcases/scene03_sphere.txt output/scene031.bmp 50
# build/PA1 testcases/scene04_axes.txt output/scene04.bmp
# build/PA1 testcases/scene05_bunny_200.txt output/scene05.bmp
# build/PA1 testcases/scene06_bunny_1k.txt output/scene06.bmp
# build/PA1 testcases/scene07_earth.txt output/scene07.bmp 200
# build/PA1 testcases/scene11_box.txt output/scene11.bmp 1000
# build/PA1 testcases/scene12_bigball.txt output/scene12.bmp 50
# build/PA1 testcases/scene13_bunnyinbox.txt output/scene13.bmp 50
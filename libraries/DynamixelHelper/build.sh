#!/bin/bash
if [ ! -d "./build/" ]; then
    mkdir ./build/
fi

cd ./build/
cmake ..

if [ "$1" = "install" ]; then
    sudo cmake --build . --target install
    sudo ldconfig
else
    cmake --build .
fi

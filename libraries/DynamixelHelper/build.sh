#!/bin/bash
if [ ! -d "./build/" ]; then
    mkdir ./build/
fi

cd ./build/
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..

if [ "$1" = "install" ]; then
    sudo cmake --build . --target install
    sudo ldconfig
else
    cmake --build .
fi

cd ../
if [ ! -f "./compile_commands.json" ]; then
    ln -s $PWD/build/compile_commands.json .
fi

#!/bin/bash
if [ ! -d "./build/" ]; then
    mkdir ./build/
fi

cd ./build/
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..
cmake --build .

cd ../
if [ ! -f "./compile_commands.json" ]; then
    ln -s $PWD/build/compile_commands.json .
fi

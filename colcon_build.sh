#!/bin/bash
colcon build --symlink-install  --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON

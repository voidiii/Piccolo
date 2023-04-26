@echo off

cmake -S . -B build -D ENABLE_PHYSICS_DEBUG_RENDERER=ON
cmake --build build --config Release

pause
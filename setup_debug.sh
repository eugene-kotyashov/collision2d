rm CMakeUserPresets.json
rm -rf build_debug
conan install . --output-folder=build_debug --build=missing --settings=build_type=Debug
cd build_debug
cmake .. -DCMAKE_TOOLCHAIN_FILE=build/Debug/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug
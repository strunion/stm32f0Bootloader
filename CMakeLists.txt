cmake_minimum_required(VERSION 3.10)
project(MultiLevelProject C)

cmake -DCMAKE_C_COMPILER=arm-none-eabi-gcc

# Set the C standard (optional)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add subdirectories for src and tests
add_subdirectory(su_f0_common)
add_subdirectory(src)

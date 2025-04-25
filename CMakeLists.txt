cmake_minimum_required(VERSION 3.10)

project(a2m12_mavlink_worker)

set(CMAKE_CXX_STANDARD 17)

find_package(MAVSDK REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} 
    PRIVATE
        ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/include
        ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src
        ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/hal
        ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/dataunpacker
        ${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/src/arch
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        MAVSDK::mavsdk
        ${CMAKE_SOURCE_DIR}/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a
        pthread
        stdc++
)

target_compile_options(${PROJECT_NAME} 
    PRIVATE 
        -Wall 
        -Wextra 
        -g
)
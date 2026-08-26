# Goes on the OrangePi 5

cmake_minimum_required(VERSION 3.10)
project(ZenithVision)

set(CMAKE_CXX_STANDARD 17)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Link RKNN and WPILib components
set(RKNN_LIB "rknnrt")
set(WPI_LIBS "ntcore" "wpiutil" "wpinet")

add_executable(zenith_vision main.cpp)
target_link_libraries(zenith_vision ${OpenCV_LIBS} ${RKNN_LIB} ${WPI_LIBS} pthread)

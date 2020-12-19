cmake_minimum_required(VERSION 3.5)
project(rplidar)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# RPLidar library
set(RPLIDAR_DRIVER_PATH "./driver")

FILE(GLOB RPLIDAR_DRIVER 
  "${RPLIDAR_DRIVER_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_DRIVER_PATH}/src/hal/*.cpp"
  "${RPLIDAR_DRIVER_PATH}/src/*.cpp"
)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_executable(rplidar_pub src/rplidar_pub.cpp ${RPLIDAR_DRIVER})
target_include_directories(rplidar_pub
  PUBLIC
    ${RPLIDAR_DRIVER_PATH}/include
    ${RPLIDAR_DRIVER_PATH}/src
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rplidar_pub rclcpp std_msgs Threads::Threads)

install(TARGETS
  rplidar_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

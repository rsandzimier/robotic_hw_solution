cmake_minimum_required(VERSION 3.5)
project(robotic_hw_solution)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/Sensor.srv"
    DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Create Cpp executable
add_executable(sensor_service_client src/SensorServiceClient.cpp)
ament_target_dependencies(sensor_service_client rclcpp)
rosidl_target_interfaces(sensor_service_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include Cpp "include" directory
# include_directories(include)

# Install Cpp executables
install(TARGETS
    sensor_service_client
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
    src/SensorServiceServer.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

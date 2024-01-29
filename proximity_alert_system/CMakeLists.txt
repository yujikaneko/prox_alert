cmake_minimum_required(VERSION 3.5)
project(proximity_alert_system)

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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Uncomment the following section to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedID.msg"
  "msg/EstimatedDistance.msg"
)

install(
  PROGRAMS
  proximity_alert_system/alert_sender.py
  proximity_alert_system/detected_id_publisher.py
  tools/receive.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.10)
project(go2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unitree_api REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegSensor.msg"
  DEPENDENCIES std_msgs unitree_go
)

# add_executable(driver src/driver.cpp)
# add_executable(odom_tf_node src/odom_tf.cpp)
add_executable(joint_state_pub_node src/joint_state_pub.cpp)
# add_executable(footprint_to_link src/footprint_to_link.cpp)
add_executable(lowstate_driver src/lowstate_driver.cpp)
add_executable(cloud_time_sync src/cloud_time_sync.cpp)
add_executable(head_camera src/head_camera.cpp)
add_executable(sportstate_cmd_vel src/sportstate_cmd_vel.cpp)

# target_include_directories(driver PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  joint_state_pub_node
  "rclcpp"
  "unitree_go"
  "sensor_msgs"
)

ament_target_dependencies(
  lowstate_driver
  "rclcpp"
  "unitree_go"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "sensor_msgs"
  "std_msgs"
)
rosidl_target_interfaces(lowstate_driver
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


ament_target_dependencies(
  cloud_time_sync
  "rclcpp"
  "sensor_msgs"
)

# OpenCV dependency for head_camera cuda
set(OpenCV_DIR "/home/unitree/.local/opencv-4.12.0-cuda/lib/cmake/opencv4")
# OpenCV dependency for head_camera system
# set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
find_package(OpenCV REQUIRED)
ament_target_dependencies(
  head_camera
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(head_camera ${OpenCV_LIBS})

# Define 3rdparty path, allowing override via environment variable or CMake variable
if(DEFINED ENV{UNITREE_3RDPARTY_PATH})
  set(UNITREE_3RDPARTY_PATH $ENV{UNITREE_3RDPARTY_PATH})
elseif(NOT DEFINED UNITREE_3RDPARTY_PATH)
  set(UNITREE_3RDPARTY_PATH "/home/unitree/work_flow/3rdparty")
endif()

ament_target_dependencies(
  sportstate_cmd_vel
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "geometry_msgs"
)
target_include_directories(sportstate_cmd_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "${UNITREE_3RDPARTY_PATH}/include")

install(TARGETS 
  # driver
  # footprint_to_link
  lowstate_driver
  # odom_tf_node
  joint_state_pub_node
  # sportstate_to_imu
  cloud_time_sync
  head_camera
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.10)
project(go2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(DEFINED ENV{UNITREE_3RDPARTY_PATH})
  set(UNITREE_3RDPARTY_PATH $ENV{UNITREE_3RDPARTY_PATH})
elseif(NOT DEFINED UNITREE_3RDPARTY_PATH)
  set(UNITREE_3RDPARTY_PATH "/home/unitree/work_flow/3rdparty")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegSensor.msg"
  DEPENDENCIES std_msgs
)

include_directories(
	include
	${UNITREE_3RDPARTY_PATH}/include
)

add_executable(lowstate_driver src/lowstate_driver.cpp)
add_executable(head_camera src/head_camera.cpp)
add_executable(sport_driver src/sport_driver.cpp)

ament_target_dependencies(
  lowstate_driver
  "rclcpp"
  "unitree_go"
  "sensor_msgs"
  "std_msgs"
)

if($ENV{ROS_DISTRO} MATCHES "foxy")
  rosidl_target_interfaces(lowstate_driver
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(lowstate_driver "${cpp_typesupport_target}")
endif()

if($ENV{ROS_DISTRO} MATCHES "foxy")
  if(DEFINED ENV{OpenCV_DIR})
    set(OpenCV_DIR $ENV{OpenCV_DIR})
  elseif(EXISTS "/home/unitree/.local/opencv-4.12.0-cuda/lib/cmake/opencv4")
    set(OpenCV_DIR "/home/unitree/.local/opencv-4.12.0-cuda/lib/cmake/opencv4")
  endif()
endif()

find_package(OpenCV REQUIRED)

ament_target_dependencies(
  head_camera
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(head_camera ${OpenCV_LIBS})

ament_target_dependencies(
  sport_driver
  "rclcpp"
  "unitree_api"
  "unitree_go"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
)

install(TARGETS
  lowstate_driver
  head_camera
  sport_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

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

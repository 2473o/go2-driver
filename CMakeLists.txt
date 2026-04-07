cmake_minimum_required(VERSION 3.10)
project(go2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
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
find_package(tf2_sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# for pointcloud 2 scan
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_components REQUIRED)

# go2_driver/msg/LegSensor.msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegSensor.msg"
  DEPENDENCIES std_msgs
)

if(DEFINED ENV{UNITREE_3RDPARTY_PATH})
  set(UNITREE_3RDPARTY_PATH $ENV{UNITREE_3RDPARTY_PATH})
elseif(NOT DEFINED UNITREE_3RDPARTY_PATH)
  set(UNITREE_3RDPARTY_PATH "/home/unitree/work_flow/3rdparty")
endif()

# add json parser
include_directories(
	include
	${UNITREE_3RDPARTY_PATH}/include
)

add_executable(lowstate_driver src/lowstate_driver.cpp)



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


# Head Camera
find_package(OpenCV REQUIRED)

add_executable(head_camera src/head_camera.cpp)
ament_target_dependencies(
  head_camera
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(head_camera ${OpenCV_LIBS})

# Sport Driver
add_executable(sport_driver src/sport_driver.cpp)
ament_target_dependencies(
  sport_driver
  "rclcpp"
  "unitree_api"
  "unitree_go"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
)


## Pointcloud Begin
set(CUPCL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/cuPCL")
set(CUPCL_FILTER_INCLUDE_DIR "${CUPCL_ROOT}/cuFilter")
set(CUPCL_FILTER_LIBRARY "${CUPCL_ROOT}/cuFilter/lib/libcudafilter.so")

find_path(CUPCL_CUDA_INCLUDE_DIR
  cuda_runtime.h
  HINTS
    /usr/local/cuda/include
    /usr/local/cuda-11.4/include
)
find_library(CUPCL_CUDART_LIBRARY
  cudart
  HINTS
    /usr/local/cuda/lib64
    /usr/local/cuda-11.4/lib64
    /usr/local/cuda/targets/aarch64-linux/lib
    /usr/local/cuda-11.4/targets/aarch64-linux/lib
)

add_library(pointcloud_to_laserscan SHARED
  src/pointcloud/pointcloud_to_laserscan_node.cpp)
target_compile_definitions(pointcloud_to_laserscan
  PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL")
target_include_directories(pointcloud_to_laserscan
  PRIVATE
    include
)
ament_target_dependencies(pointcloud_to_laserscan
  "laser_geometry"
  "message_filters"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
)

if(EXISTS "${CUPCL_FILTER_INCLUDE_DIR}/lib/cudaFilter.h" AND
   EXISTS "${CUPCL_FILTER_LIBRARY}" AND
   CUPCL_CUDA_INCLUDE_DIR AND
   CUPCL_CUDART_LIBRARY)
  add_library(cupcl_filter SHARED IMPORTED GLOBAL)
  set_target_properties(cupcl_filter PROPERTIES
    IMPORTED_LOCATION "${CUPCL_FILTER_LIBRARY}"
  )
  add_library(cupcl_cudart SHARED IMPORTED GLOBAL)
  set_target_properties(cupcl_cudart PROPERTIES
    IMPORTED_LOCATION "${CUPCL_CUDART_LIBRARY}"
  )
  target_compile_definitions(pointcloud_to_laserscan PRIVATE GO2_PERCEPTION_HAS_CUPCL=1)
  target_include_directories(pointcloud_to_laserscan PRIVATE
    "${CUPCL_FILTER_INCLUDE_DIR}"
    "${CUPCL_CUDA_INCLUDE_DIR}"
  )
  target_link_libraries(pointcloud_to_laserscan cupcl_filter cupcl_cudart)
  set_target_properties(pointcloud_to_laserscan PROPERTIES
    BUILD_RPATH "${CUPCL_ROOT}/cuFilter/lib"
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../../lib;${CUPCL_ROOT}/cuFilter/lib"
  )
  install(FILES "${CUPCL_FILTER_LIBRARY}" DESTINATION lib)
  message(STATUS "cuPCL enabled with ${CUPCL_FILTER_LIBRARY}")
else()
  message(WARNING
    "cuPCL disabled. header=${CUPCL_FILTER_INCLUDE_DIR}/lib/cudaFilter.h "
    "lib=${CUPCL_FILTER_LIBRARY} cuda_include=${CUPCL_CUDA_INCLUDE_DIR} "
    "cudart=${CUPCL_CUDART_LIBRARY}")
endif()

rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

add_executable(cloud_accumulation
  src/pointcloud/cloud_accumulation.cpp
)
ament_target_dependencies(cloud_accumulation
  "laser_geometry"
  "message_filters"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
)

## Pointcloud END

install(TARGETS
  pointcloud_to_laserscan
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(TARGETS
  lowstate_driver
  head_camera
  sport_driver
  cloud_accumulation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

####################### Python Scripts
install(PROGRAMS
  scripts/multi_static_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

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

cmake_minimum_required(VERSION 3.5)
project(ros_bridge)
find_package(Protobuf REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/misc/myprotobuf.cmake)
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/misc/*.proto")
my_protobuf_generate_cpp(generated_files/myproject ProtoSources ProtoHeaders ${ProtoFiles})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated_files/myproject)
include_directories(${Protobuf_INCLUDE_DIRS})

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
include_directories(${JSONCPP_INCLUDE_DIRS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(webots_controller src/webots_controller.cpp src/robot_client/robot_client.cpp ${ProtoSources} ${ProtoHeaders})
target_include_directories(webots_controller PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(webots_controller rclcpp std_msgs rosgraph_msgs cv_bridge image_transport Protobuf OpenCV JSONCPP)

install(TARGETS webots_controller
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

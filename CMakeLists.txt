cmake_minimum_required(VERSION 3.5)
project(fast_limo)

# Silence CMP0074 warning (PCL_ROOT etc.)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# -------------------------
# Compiler / build settings
# -------------------------
add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE "Release")

# -------------------------
# Dependencies
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)

find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "OpenMP not found; building without OpenMP.")
endif()

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

# -------------------------
# ROS 2 interfaces (srv)
# -------------------------
find_package(rosidl_default_generators REQUIRED)

# NOTE: This creates a custom target named *fast_limo* (the package name).
# Do NOT also create a C++ target with the same name.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  DEPENDENCIES builtin_interfaces
)

# -------------------------
# Includes
# -------------------------
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

message(STATUS "PCL version: ${PCL_VERSION}")

# -------------------------
# IKFOM lib
# -------------------------
add_library(ikfom
  include/IKFoM/IKFoM_toolkit
  include/IKFoM/use-ikfom.cpp
)
target_link_libraries(ikfom ${PCL_LIBRARIES} ${OpenMP_LIBS})
target_include_directories(ikfom PUBLIC include/IKFoM/ include/IKFoM/IKFoM_toolkit)

# -------------------------
# fast_limo core lib  (renamed from ${PROJECT_NAME} -> fast_limo_core)
# -------------------------
add_library(fast_limo_core
  include/fast_limo/Modules/Localizer.cpp
  include/fast_limo/Modules/Mapper.cpp
  include/fast_limo/Objects/State.cpp
  include/fast_limo/Objects/Plane.cpp
  include/fast_limo/Objects/Match.cpp
  include/fast_limo/Objects/Octree.hpp
)
if(OPENMP_FOUND)
  target_compile_options(fast_limo_core PRIVATE ${OpenMP_FLAGS})
endif()
target_link_libraries(fast_limo_core ${PCL_LIBRARIES} ${OpenMP_LIBS} ikfom)

# -------------------------
# Executables
# -------------------------
add_executable(${PROJECT_NAME}_multi_exec
  src/LimoWrapper.cpp
)
ament_target_dependencies(${PROJECT_NAME}_multi_exec
  rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros pcl_conversions std_srvs
)
target_link_libraries(${PROJECT_NAME}_multi_exec fast_limo_core ${PCL_LIBRARIES})

# Make generated service headers visible to this executable
target_include_directories(${PROJECT_NAME}_multi_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
# Ensure interfaces are generated before compiling the exec
add_dependencies(${PROJECT_NAME}_multi_exec ${PROJECT_NAME}__rosidl_generator_cpp)
# Some toolchains benefit from explicit typesupport linkage
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_multi_exec ${cpp_typesupport_target})

# (Optional) Accumulator node — keep if you still use it
add_executable(accumulate_map_node
  src/accumulate_map_node.cpp
)
ament_target_dependencies(accumulate_map_node
  rclcpp sensor_msgs nav_msgs std_srvs pcl_conversions
)
target_link_libraries(accumulate_map_node ${PCL_LIBRARIES})
target_include_directories(accumulate_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
add_dependencies(accumulate_map_node ${PROJECT_NAME}__rosidl_generator_cpp)
target_link_libraries(accumulate_map_node ${cpp_typesupport_target})

# Odometry saver node
add_executable(odometry_saver
  src/odometry_saver.cpp
)
ament_target_dependencies(odometry_saver
  rclcpp nav_msgs
)

# -------------------------
# Install
# -------------------------
install(TARGETS
  ${PROJECT_NAME}_multi_exec
  accumulate_map_node
  odometry_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Export runtime deps for the srv
ament_export_dependencies(rosidl_default_runtime)

ament_package()

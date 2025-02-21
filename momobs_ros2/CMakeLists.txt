cmake_minimum_required(VERSION 3.8)
project(momobs_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(DEPS
  rclcpp
  pinocchio
  Eigen3
  std_msgs
  observer_msgs
  momobs
  sensor_msgs
  nav_msgs
  message_filters
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()

message(STATUS "Check for anymal_msgs...")
#BAG WRAPPER
find_package(anymal_msgs QUIET)

if(anymal_msgs_FOUND)
    add_executable(momobs_bag 
      src/bag_wrapper.cpp
      src/wrapper_base.cpp
    )
    ament_target_dependencies(momobs_bag 
    ${DEPS} 
    anymal_msgs)
    install(
        TARGETS momobs_bag
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
else()
    message(WARNING "anymal_msgs not found, skipping momobs_bag wrapper")
endif()



message(STATUS "Check for mujoco_msgs...")
#MUJOCO WRAPPER
find_package(mujoco_msgs QUIET)

if(mujoco_msgs_FOUND)
    add_executable(momobs_mujoco
      src/mujoco_wrapper.cpp
      src/wrapper_base.cpp
    )
    ament_target_dependencies(momobs_mujoco
    ${DEPS} 
    mujoco_msgs)
    install(
        TARGETS momobs_mujoco
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
else()
    message(WARNING "mujoco_msgs not found, skipping momobs_mujoco wrapper")
endif()



if (NOT anymal_msgs_FOUND AND NOT mujoco_msgs_FOUND)
  message(FATAL_ERROR "No wrapper executabl was compiled")
endif()





include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
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

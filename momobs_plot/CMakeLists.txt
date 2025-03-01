cmake_minimum_required(VERSION 3.8)
project(momobs_plot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY theme res
DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS  ${PROJECT_NAME}/residual_plotter.py 
            ${PROJECT_NAME}/force_plotter.py 
            ${PROJECT_NAME}/friction_setter.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

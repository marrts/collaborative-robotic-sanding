cmake_minimum_required(VERSION 3.5)
project(crs_application)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-pedantic -Wno-missing-field-initializers)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(crs_msgs REQUIRED)
find_package(scxml_core REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED NO_MODULE)

##################################
# build
##################################
add_library(${PROJECT_NAME} 
	src/crs_executive.cpp 
	src/task_managers/scan_acquisition_manager.cpp
	src/task_managers/part_registration_manager.cpp
	src/task_managers/part_rework_manager.cpp
	src/task_managers/motion_planning_manager.cpp
	src/task_managers/process_execution_manager.cpp)
target_link_libraries(${PROJECT_NAME} 
  scxml_core::scxml_core
  Eigen3::Eigen
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components crs_msgs)

add_executable(${PROJECT_NAME}_node src/crs_application.cpp)
target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp_components crs_msgs)

##################################
# install
##################################
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})
install(TARGETS ${PACKAGE_LIBRARIES}
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION lib)

install(TARGETS ${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY resources launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
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

cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(scenario_test_runner)

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config
            ${CMAKE_CURRENT_SOURCE_DIR}/launch
            ${CMAKE_CURRENT_SOURCE_DIR}/scenario
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest_REQUIRED)
#  find_package(launch_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(pytest test)
endif()

ament_package()

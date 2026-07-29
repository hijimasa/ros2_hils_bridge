cmake_minimum_required(VERSION 3.8)
project(hils_bridge_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/scenario/scenario_runner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scenario_runner
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fault_injection test/test_fault_injection.py)
  ament_add_pytest_test(test_scenario test/test_scenario.py)
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(hils_bridge_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InjectFault.srv"
  "srv/ClearFault.srv"
  "srv/GetFaultState.srv"
  "srv/LoadScenario.srv"
  "srv/GetScenarioState.srv"
  "srv/SetDeviceState.srv"
  "srv/GetDeviceState.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(cognitive_framework)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY
  cognitive_framework
  DESTINATION lib/python3.12/site-packages/
)

# Install Python executables
install(PROGRAMS
  cognitive_framework/sense_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sense_node
)
install(PROGRAMS
  cognitive_framework/memory_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME memory_node
)
install(PROGRAMS
  cognitive_framework/action_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME action_node
)
install(PROGRAMS
  cognitive_framework/action_node_with_llm.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME action_node_with_llm
)
install(PROGRAMS
  cognitive_framework/action_display_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME action_display_node
)
install(PROGRAMS
  cognitive_framework/llm_service_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME llm_service_node
)
install(PROGRAMS
  cognitive_framework/single_llm_action_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME single_llm_action_example
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "cognitive_framework/srv/MemoryRequest.srv"
  DEPENDENCIES std_msgs
)

ament_package() 
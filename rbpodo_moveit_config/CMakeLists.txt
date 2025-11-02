cmake_minimum_required(VERSION 3.22)
project(rbpodo_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# === 3. worlds 디렉토리 설치 (이게 핵심!) ===
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.world" PATTERN "*.sdf"
)
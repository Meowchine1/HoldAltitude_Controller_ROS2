cmake_minimum_required(VERSION 3.8)
project(control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Путь к MAVSDKConfig.cmake (если MAVSDK установлен нестандартно)
set(MAVSDK_DIR "/usr/lib/cmake/MAVSDK")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(MAVSDK REQUIRED)

add_executable(control_node src/control.cpp)
target_include_directories(control_node PRIVATE /usr/include/mavsdk)

ament_target_dependencies(control_node rclcpp MAVSDK)

# Линкуем MAVSDK явно
target_link_libraries(control_node MAVSDK::mavsdk)

install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Устанавливаем папку launch с файлом запуска
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/processes.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

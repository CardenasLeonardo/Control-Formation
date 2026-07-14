cmake_minimum_required(VERSION 3.5)
project(articubot_one)

# ----------------------------------------------------
# Default standards
# ----------------------------------------------------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------
# Dependencies
# ----------------------------------------------------
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ----------------------------------------------------
# 🟦 Install launch/ directory
# ----------------------------------------------------
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ----------------------------------------------------
# 🟩 Install description/ directory (URDF + XACRO)
# ----------------------------------------------------
install(
  DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

# ----------------------------------------------------
ament_package()

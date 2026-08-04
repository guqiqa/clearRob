cmake_minimum_required(VERSION 3.16)
project(chassis_core)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# C++17 for std::clamp, std::optional, etc.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)

pybind11_add_module(chassis_core
  src/pybind_module.cpp
  src/chassis_core.cpp
)

target_include_directories(chassis_core PRIVATE include)

target_compile_options(chassis_core PRIVATE -Wall -Wextra -O2)

# Install the compiled .so into the site-packages root so `import chassis_core`
# finds it directly (a sub-directory would shadow it as a namespace package).
install(TARGETS chassis_core
  DESTINATION lib/python3.10/site-packages
)

ament_package()

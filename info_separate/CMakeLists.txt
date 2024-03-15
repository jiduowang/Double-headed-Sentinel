cmake_minimum_required(VERSION 3.8)
project(info_separate)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)

# Auto-generate the build and install rules
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/camera_info_seprate.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# Add the executable target
add_executable(info_seprate_node src/camera_info_seprate.cpp)

# Link the library to the executable
target_link_libraries(info_seprate_node ${PROJECT_NAME})

# Install the executable
install(TARGETS
  info_seprate_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

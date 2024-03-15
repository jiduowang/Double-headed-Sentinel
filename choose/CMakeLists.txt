cmake_minimum_required(VERSION 3.8)
project(choose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)

# Auto-generate the build and install rules
ament_auto_find_build_dependencies()

# Add the library target
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/choose.cpp
)

# Add the include directories and link the libraries
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# Add the executable target
add_executable(choose_node src/choose.cpp)

# Link the library to the executable
target_link_libraries(choose_node ${PROJECT_NAME})

# Install the executable
install(TARGETS
  choose_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, configuration, and other resources


# Generate the ament index
ament_auto_package()
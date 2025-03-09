cmake_minimum_required(VERSION 3.5)
project(fire_fighting)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(psdk_interfaces REQUIRED)  # For the service interface
find_package(OpenCV 4 REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${psdk_interfaces_INCLUDE_DIRS}
)

# Add executable
add_executable(video_compress src/video_compress.cpp) # Replace src/my_node.cpp with your source file

# Link dependencies
ament_target_dependencies(video_compress
        rclcpp
        std_msgs
        "rclcpp_lifecycle"
        "lifecycle_msgs"
        cv_bridge
        sensor_msgs
        psdk_interfaces
        image_transport
) # Add other dependencies as needed

# Link OpenCV
target_link_libraries(video_compress ${OpenCV_LIBS})

# Install the executable (optional, but recommended)
install(TARGETS
        video_compress
        DESTINATION lib/${PROJECT_NAME}
)

# Install the include directory (if you have header files)
# install(DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include/${PROJECT_NAME}
# )

ament_package()

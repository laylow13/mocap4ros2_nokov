cmake_minimum_required(VERSION 3.5)

project(mocap4r2_nokov_driver)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_control REQUIRED)

set(dependencies
        rclcpp
        rclcpp_lifecycle
        mocap4r2_msgs
        mocap4r2_control
        geometry_msgs
)
include_directories(
        include
        deps/nokov_sdk/include)
add_library(${PROJECT_NAME} src/mocap4r2_nokov_driver/mocap4r2_nokov_driver.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# link nokov sdk
set (nokov_sdk_LIB_DIRS deps/nokov_sdk/lib)
if (${ARCHITECTURE} MATCHES "arm")
    set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/armhf/libnokov_sdk.so)
elseif (${ARCHITECTURE} MATCHES "aarch")
    set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/aarch64/libnokov_sdk.so)
else ()
    set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/x64/libnokov_sdk.so)
endif ()

message(STATUS ${nokov_sdk_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${nokov_sdk_LIBRARIES})

add_executable(mocap4r2_nokov_driver_main src/mocap4r2_nokov_driver_main.cpp)
target_link_libraries(mocap4r2_nokov_driver_main  ${PROJECT_NAME} )
ament_target_dependencies(mocap4r2_nokov_driver_main ${dependencies})


install(DIRECTORY include/
        DESTINATION include/
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        mocap4r2_nokov_driver_main
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if (${ARCHITECTURE} MATCHES "arm")
    install(DIRECTORY deps/nokov_sdk/include deps/nokov_sdk/lib/armhf
            DESTINATION share/${PROJECT_NAME})
elseif (${ARCHITECTURE} MATCHES "aarch")
    install(DIRECTORY deps/nokov_sdk/include deps/nokov_sdk/lib/aarch64
            DESTINATION share/${PROJECT_NAME})
else ()
    install(DIRECTORY deps/nokov_sdk/include deps/nokov_sdk/lib/x64
            DESTINATION share/${PROJECT_NAME})
endif ()

#install(TARGETS
#        ${PROJECT_NAME} vicon_sdk
#        ARCHIVE DESTINATION lib
#        LIBRARY DESTINATION lib
#        RUNTIME DESTINATION bin
#)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

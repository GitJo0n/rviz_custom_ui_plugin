cmake_minimum_required(VERSION 3.0.2)
project(rviz_custom_ui_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  interactive_markers
  rviz
  visualization_msgs
  # yolo
  image_transport
  cv_bridge
  sensor_msgs
  yolov10_ros_msgs
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)  # Qt5
find_package(OpenCV REQUIRED)    # yolo

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}  # Qt5 include
)

add_definitions(${Qt5Widgets_DEFINITIONS})  # Qt5

# 독립 실행형 UI 실행 파일 추가
add_executable(rviz_custom_ui src/main.cpp)
target_link_libraries(rviz_custom_ui ${catkin_LIBRARIES} Qt5::Widgets)
add_executable(marker_node src/marker_node.cpp)
target_link_libraries(marker_node ${catkin_LIBRARIES} Qt5::Widgets)

# 독립 실행형 실행 파일 설치
install(TARGETS rviz_custom_ui marker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


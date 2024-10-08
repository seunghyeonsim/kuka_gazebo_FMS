cmake_minimum_required(VERSION 3.8)
project(kuka_control_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kuka_control_box_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)  # 추가: trajectory_msgs 패키지 찾기

include_directories(include)

# Robotics 라이브러리 생성
add_library(Robotics src/Robotics.cpp)
ament_target_dependencies(Robotics rclcpp Eigen3)

# kuka_ik_service_server 실행 파일 설정
add_executable(kuka_ik_service_server src/kuka_ik_service_server.cpp)
target_link_libraries(kuka_ik_service_server Robotics)
ament_target_dependencies(kuka_ik_service_server 
  rclcpp 
  Eigen3 
  kuka_control_box_srvs 
  nav_msgs 
  geometry_msgs 
  trajectory_msgs)  # 추가: trajectory_msgs 의존성 추가

# 설치 경로 설정
install(TARGETS
  kuka_ik_service_server
  Robotics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

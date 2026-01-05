# Gerekli minimum CMake versiyonunu ve proje adını belirtiyoruz.
cmake_minimum_required(VERSION 3.8)
project(pointcloud_transformer)

# ROS 2 Humble için varsayılan olan C++17 standardını kullanıyoruz.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Derleyici uyarılarını etkinleştiriyoruz.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake ve diğer ROS 2 paketlerini buluyoruz.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# src klasöründeki C++ dosyasından bir "çalıştırılabilir dosya" oluşturuyoruz.
# Orijinal dosya adınız 'pointcloud_transformer_nodelet.cpp' ise
# onu 'pointcloud_transformer_node.cpp' gibi bir isme çevirmeniz daha uygun olur.
add_executable(pointcloud_transformer_node src/pointcloud_transformer_node.cpp)
add_executable(pose_transformer_node src/pose_transformer_node.cpp)

# Çalıştırılabilir dosyamızın hangi ROS 2 kütüphanelerine bağlı olduğunu belirtiyoruz.
# Bu komut, gerekli başlık dosyalarını (include) ve kütüphaneleri (link) otomatik olarak bağlar.
ament_target_dependencies(pointcloud_transformer_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions)

ament_target_dependencies(pose_transformer_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs)

# Derlenen çalıştırılabilir dosyayı 'install/pointcloud_transformer/lib' altına kuruyoruz.
# Bu, 'ros2 run' komutuyla düğümümüzü bulabilmemiz için gereklidir.
install(TARGETS

  pointcloud_transformer_node
  pose_transformer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Paketi ament sistemiyle tamamlıyoruz.
ament_package()

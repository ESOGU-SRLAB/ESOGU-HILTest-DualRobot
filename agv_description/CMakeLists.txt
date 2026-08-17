cmake_minimum_required(VERSION 3.8)
project(agv_description)

# ROS 2 yapı sistemi
find_package(ament_cmake REQUIRED)

# İleride başka bağımlılıklar eklersen buraya yazabilirsin:
# find_package(ros_gz_sim REQUIRED)

# Mesh, URDF ve Launch dosyalarını kurulum dizinine kopyala
install(
  DIRECTORY
    meshes
    urdf
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()

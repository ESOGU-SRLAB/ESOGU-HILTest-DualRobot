cmake_minimum_required(VERSION 3.0.2)
project(onrobot_api)

set(ONROBOT_API_VERSION 0.0.0)

add_compile_options(-std=c++17)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(modbus REQUIRED libmodbus)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED src/onrobot_api.cpp src/onrobot_api_implementation.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads ${modbus_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${modbus_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PUBLIC ${modbus_CFLAGS_OTHER})

add_executable(onrobot_example src/example.cpp)
  target_link_libraries(onrobot_example ${PROJECT_NAME})
  target_include_directories(onrobot_example PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_executable(onrobot_cli src/cli.cpp)
  target_link_libraries(onrobot_cli ${PROJECT_NAME})
  target_include_directories(onrobot_cli PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

set(INCLUDE_INSTALL_DIR include/ CACHE PATH INTERNAL)
set(BIN_INSTALL_DIR bin/ CACHE PATH INTERNAL)
set(LIB_INSTALL_DIR lib/ CACHE PATH INTERNAL)
set(SAHRE_INSTALL_DIR share/ CACHE PATH INTERNAL)

# Install
install(TARGETS ${PROJECT_NAME} onrobot_example onrobot_cli
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

include(CMakePackageConfigHelpers)

configure_package_config_file(cmake/onrobot_apiConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/onrobot_apiConfig.cmake
  INSTALL_DESTINATION ${SAHRE_INSTALL_DIR}/cmake/onrobot_api
  PATH_VARS INCLUDE_INSTALL_DIR ONROBOT_API_VERSION)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/onrobot_apiConfigVersion.cmake
  VERSION ${ONROBOT_API_VERSION} COMPATIBILITY SameMajorVersion )

install(EXPORT ${PROJECT_NAME} DESTINATION "${SAHRE_INSTALL_DIR}/cmake/onrobot_api")

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/onrobot_apiConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/onrobot_apiConfigVersion.cmake
        DESTINATION ${SAHRE_INSTALL_DIR}/cmake/onrobot_api )
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/rs_driver/src
)

add_executable(rslidar_driver rslidar_driver.cc )
target_link_libraries(rslidar_driver
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
  pthread
  pcap
)


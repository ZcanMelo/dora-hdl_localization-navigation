

include_directories(
  ${DORA_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(vehicle_chassis_node can_protocol.cpp msg_interface.cpp socketcan.cpp v_chassis_interface.cpp)
target_link_libraries(vehicle_chassis_node
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
  pthread
)





add_library(CameraTrigger CameraTrigger.cpp)
add_dependencies(CameraTrigger autorally_msgs_gencpp)
target_link_libraries(CameraTrigger ${catkin_LIBRARIES} ${Boost_LIBRARIES} SerialSensorInterface Diagnostics)

install(TARGETS
  CameraTrigger
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

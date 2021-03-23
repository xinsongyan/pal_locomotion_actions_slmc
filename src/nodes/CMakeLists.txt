

add_executable(push_walking_action_slmc push_walking_action_slmc.cpp)
target_link_libraries(push_walking_action_slmc ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS
      push_walking_action_slmc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

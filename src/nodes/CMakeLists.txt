add_executable(push_icp_ds_action push_icp_ds_action.cpp)
target_link_libraries(push_icp_ds_action ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_stand_one_leg_action push_stand_one_leg_action.cpp)
target_link_libraries(push_stand_one_leg_action ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_stand_down_leg_action push_stand_down_leg_action.cpp)
target_link_libraries(push_stand_down_leg_action ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(move_swing_leg_waypoints move_swing_leg_waypoints.cpp)
target_link_libraries(move_swing_leg_waypoints ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_com_action push_com_action.cpp)
target_link_libraries(push_com_action ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_csv_com_action push_csv_com_action.cpp)
target_link_libraries(push_csv_com_action ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS move_swing_leg_waypoints push_stand_one_leg_action push_stand_down_leg_action push_icp_ds_action push_com_action push_csv_com_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

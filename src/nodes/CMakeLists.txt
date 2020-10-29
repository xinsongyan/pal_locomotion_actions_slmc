add_executable(push_csv_com_action push_csv_com_action.cpp)
target_link_libraries(push_csv_com_action ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_csv_walking_action push_csv_walking_action.cpp)
target_link_libraries(push_csv_walking_action ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_csv_walking_action_prev push_csv_walking_action_prev.cpp)
target_link_libraries(push_csv_walking_action_prev ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_icp_ds_action push_icp_ds_action.cpp)
target_link_libraries(push_icp_ds_action ${PROJECT_NAME} ${catkin_LIBRARIES})


install(TARGETS push_csv_com_action
        push_csv_walking_action
        push_csv_walking_action_prev
        push_icp_ds_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

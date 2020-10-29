set(SRC ${SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}/csv_com_action.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/csv_walking_action.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/csv_walking_action_prev.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/move_icp_ds_action.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/stand_one_leg_action.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/stand_down_leg_action.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/single_leg_stand_action.cpp
        PARENT_SCOPE)

add_library(${PROJECT_NAME}_plugins biped_walking_actions_plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME}  ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_plugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


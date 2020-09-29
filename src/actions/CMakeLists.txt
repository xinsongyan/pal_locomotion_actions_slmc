set(SRC ${SRC} ${CMAKE_CURRENT_SOURCE_DIR}/wbc_action.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/balance_ds_action.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/balancing_action.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/static_walk.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/move_icp_ds_action.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/sstand_one_leg_action.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/stand_down_leg_action.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/com_action.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/csv_com_action.cpp
               PARENT_SCOPE)

add_library(${PROJECT_NAME}_plugins biped_walking_actions_plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME}  ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_plugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


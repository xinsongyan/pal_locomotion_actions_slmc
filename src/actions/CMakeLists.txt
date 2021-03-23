set(SRC ${SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}/walking_action_slmc.cpp
        PARENT_SCOPE)

add_library(${PROJECT_NAME}_plugins biped_walking_actions_plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME}  ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_plugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


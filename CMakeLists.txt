cmake_minimum_required(VERSION 2.8.3)
project(pal_locomotion_actions_slmc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED pal_locomotion pluginlib)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS pal_locomotion
    )

include_directories(
    include
    )
include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    )

add_subdirectory(src/actions)
add_subdirectory(src/nodes)

add_library(${PROJECT_NAME} src/icp_control_utils.cpp
                            ${SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(start_wbc_action src/start_wbc_action.cpp)
#target_link_libraries(start_wbc_action ${catkin_LIBRARIES})

add_executable(add_two_ints_server src/service/trigger_server_node.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    )

install(FILES biped_walking_actions.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

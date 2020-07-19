add_executable(ompl_constrained_planning src/ompl_constrained_planning.cpp)
target_link_libraries(ompl_constrained_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ompl_constrained_planning DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
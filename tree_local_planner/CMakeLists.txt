cmake_minimum_required(VERSION 2.8.3)
project(tree_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            pcl_conversions
            roscpp
            tf
            ompl_visual_tools
        )

find_package(OMPL REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
    
add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/TreePlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        dynamic_reconfigure
        pluginlib
        roscpp
)

#add_library(${PROJECT_NAME} src/dwa_planner.cpp src/dwa_planner_ros.cpp)
add_library(${PROJECT_NAME} src/tree_planner_ros.cpp src/ompl_visualization.cpp)
target_link_libraries(${PROJECT_NAME} base_local_planner ${OMPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} tree_local_planner_gencfg)
add_dependencies(${PROJECT_NAME} nav_msgs_gencpp)

install(TARGETS ${PROJECT_NAME}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

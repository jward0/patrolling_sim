cmake_minimum_required(VERSION 2.8.3)
project(patrolling_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  nav_msgs
  roscpp
#  rospy
  roslib
  tf
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GoToStartPosSrv.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS src
   LIBRARIES PatrolAgent SSIPatrolAgent
#  CATKIN_DEPENDS actionlib move_base_msgs nav_msgs roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(PatrolAgent
  src/PatrolAgent.cpp
  src/getgraph.cpp
  src/algorithms.cpp
  src/config.cpp
)

# Declare a cpp library
add_library(SSIPatrolAgent
  src/SSIPatrolAgent.cpp
)

## Declare a cpp executable
add_executable(Conscientious_Reactive src/Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
 
## Declare a cpp executable
add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Heuristic_Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
#  
## Declare a cpp executable 
add_executable(Conscientious_Cognitive src/Conscientious_Cognitive_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Conscientious_Cognitive PatrolAgent ${catkin_LIBRARIES})
 
## Declare a cpp executable 
add_executable(Cyclic src/Cyclic_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Cyclic PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(MSP src/MSP_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(MSP PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(GBS src/GBS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(GBS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(SBS src/SBS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(SBS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(SEBS src/SEBS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(SEBS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(S_SEBS src/S_SEBS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(S_SEBS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
# add_executable(R_SEBS src/R_SEBS_Agent.cpp)
## Specify libraries to link a library or executable target against
# target_link_libraries(R_SEBS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(SPNS src/SPNS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(SPNS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(MNS src/MNS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(MNS PatrolAgent ${catkin_LIBRARIES})

add_executable(ER src/ER_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(ER PatrolAgent ${catkin_LIBRARIES})

add_executable(RHARM src/RHARM_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(RHARM PatrolAgent ${catkin_LIBRARIES})

add_executable(CRA src/CRA_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(CRA PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(CBLS src/CBLS_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(CBLS PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(Random src/Random_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(Random PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(DTAGreedy src/DTAGreedy_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(DTAGreedy PatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(DTASSI src/DTASSI_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(DTASSI PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(DTASSIPart src/DTASSIPart_Agent.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(DTASSIPart PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})
 
 
# Results and Monitor Node:
 
## Declare a cpp executable 
add_executable(monitor src/monitor.cpp src/getgraph.cpp)
add_dependencies(monitor ${PROJECT_NAME}_gencpp)
## Specify libraries to link a library or executable target against
target_link_libraries(monitor ${catkin_LIBRARIES})

add_executable(GoToStartPos src/GoToStartPos.cpp)
add_dependencies(GoToStartPos ${PROJECT_NAME}_gencpp)
target_link_libraries(GoToStartPos ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(idlHistogram src/idlHistogram.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(idlHistogram ${catkin_LIBRARIES})


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(patrolling_sim_node patrolling_sim_generate_messages_cpp)


 



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS patrolling_sim patrolling_sim_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_patrolling_sim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#add_executable(Conscientious_Reactive src/Conscientious_Reactive.cpp)
#add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive.cpp)
#add_executable(Conscientious_Cognitive src/Conscientious_Cognitive.cpp)
#add_executable(Cyclic src/Cyclic.cpp)
#add_executable(MSP src/MSP.cpp)
#add_executable(GBS src/GBS.cpp)
#add_executable(SEBS src/SEBS.cpp)


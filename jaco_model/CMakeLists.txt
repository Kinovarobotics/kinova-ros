cmake_minimum_required(VERSION 2.8.3)
project(jaco_model)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
   urdf/standalone_arm.xacro
   urdf/jaco_arm.xacro
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
 )
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

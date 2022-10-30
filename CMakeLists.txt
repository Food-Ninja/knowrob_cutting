cmake_minimum_required(VERSION 2.8.3)
project(knowrob_example)

find_package(catkin REQUIRED COMPONENTS
  knowrob
)

find_package(catkin REQUIRED COMPONENTS
    rosprolog roscpp roslib)

find_package(PkgConfig)

pkg_check_modules(SWIPL REQUIRED swipl)
link_directories(${SWIPL_LIBRARY_DIRS})

pkg_check_modules(CURLPP REQUIRED curlpp)
include_directories(${CURLPP_INCLUDE_DIRS})
link_directories(${CURLPP_LIBRARY_DIRS})

pkg_check_modules(JSONCPP REQUIRED jsoncpp)
include_directories(${JSONCPP_INCLUDE_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})

catkin_package(
	CATKIN_DEPENDS rosprolog roscpp roslib
)

set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")
include_directories(include ${SWIPL_INCLUDE_DIRS} ${CURLPP_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

##############
##### DB #####
##############

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)

install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
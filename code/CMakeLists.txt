cmake_minimum_required(VERSION 2.8.3)
project(mylocal)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  dynamic_reconfigure
)


#set(Boost_INCLUDE_DIR /usr/include)  
#set(Boost_LIBRARY_DIR /usr/lib/x86_64-linux-gnu)  
find_package(Boost COMPONENTS system filesystem serialization REQUIRED)  
#include_directories(${Boost_INCLUDE_DIR})  
#link_directories(${Boost_LIBRARY_DIR})  


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chapter2tutor
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(
/usr/local/include/flirtlib
)
#link_libraries(/usr/local/lib)
#link_directories(/usr/local/lib)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/chapter2tutor.cpp
# )

#add_executable(chap2_example1_a src/example1_a.cpp)
#add_executable(chap2_example1_b src/example1_b.cpp)
#add_executable(subscan_node src/subscan.cpp)
#add_executable(datasample_node src/datasample.cpp)
#add_executable(detect1_node src/detect1.cpp)
#add_executable(showinterest_node src/showinterest.cpp)
#add_executable(testinterest_node src/testinterest.cpp)
#add_executable(testboost_node src/testboost.cpp)
add_executable(boostread_node src/boostread.cpp)
add_executable(datainnersample_node src/datainnersample.cpp)
add_executable(local1_node src/local1.cpp)
add_executable(listenandpub_node src/listenandpub.cpp)
add_executable(location_node src/location.cpp)
#add_dependencies(chap2_example1_a #chapter2_tutorials_generate_messages_cpp)
#add_dependencies(chap2_example1_b #chapter2_tutorials_generate_messages_cpp)

#target_link_libraries(chap2_example1_a ${catkin_LIBRARIES})
#target_link_libraries(chap2_example1_b ${catkin_LIBRARIES})
#target_link_libraries(subscan_node ${catkin_LIBRARIES}
#libflirtlib_feature.so
#libflirtlib_geometry.so
#libflirtlib_sensors.so 
#libflirtlib_utils.so
#)
#target_link_libraries(datasample_node ${catkin_LIBRARIES})

#target_link_libraries(detect1_node ${catkin_LIBRARIES}
#libflirtlib_feature.so
#libflirtlib_geometry.so
#libflirtlib_sensors.so 
#libflirtlib_utils.so
#)
#target_link_libraries(showinterest_node ${catkin_LIBRARIES}
#libflirtlib_feature.so
#libflirtlib_geometry.so
#libflirtlib_sensors.so 
#libflirtlib_utils.so
#)
#target_link_libraries(testinterest_node ${catkin_LIBRARIES}
#libflirtlib_feature.so
#libflirtlib_geometry.so
#libflirtlib_sensors.so 
#libflirtlib_utils.so
#)
#target_link_libraries(testboost_node ${catkin_LIBRARIES}
#libflirtlib_feature.so
#libflirtlib_geometry.so
#libflirtlib_sensors.so 
#libflirtlib_utils.so
#${Boost_LIBRARIES}
#)
target_link_libraries(boostread_node ${catkin_LIBRARIES} 
libflirtlib_feature.so
libflirtlib_geometry.so
libflirtlib_sensors.so 
libflirtlib_utils.so
${Boost_LIBRARIES}
)



target_link_libraries(datainnersample_node ${catkin_LIBRARIES} 
libflirtlib_feature.so
libflirtlib_geometry.so
libflirtlib_sensors.so 
libflirtlib_utils.so
${Boost_LIBRARIES}
)

target_link_libraries(local1_node ${catkin_LIBRARIES} 
libflirtlib_feature.so
libflirtlib_geometry.so
libflirtlib_sensors.so 
libflirtlib_utils.so
${Boost_LIBRARIES}
)

target_link_libraries(listenandpub_node ${catkin_LIBRARIES} 
)

target_link_libraries(location_node ${catkin_LIBRARIES} 
libflirtlib_feature.so
libflirtlib_geometry.so
libflirtlib_sensors.so 
libflirtlib_utils.so
${Boost_LIBRARIES}
)

#
#libflirtlib_gui.so
#libflirtlib_sensorstream.so

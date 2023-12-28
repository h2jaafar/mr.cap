file (GLOB SRC_HEADER_LIST CONFIGURE_DEPENDS ${mrcap_SOURCE_DIR}/include/mrcap/*.h)
include(FetchContent)

set(SRC_HEADER_LIST
${mrcap_SOURCE_DIR}/include/mrcap/MainWindow.h
${mrcap_SOURCE_DIR}/include/mrcap/Logger.h
${mrcap_SOURCE_DIR}/include/mrcap/Plotter.h
${mrcap_SOURCE_DIR}/include/mrcap/mrcap.h
${mrcap_SOURCE_DIR}/include/mrcap/Utils.h
${mrcap_SOURCE_DIR}/include/mrcap/customFactor.h
${mrcap_SOURCE_DIR}/include/mrcap/Trajectory.h
${mrcap_SOURCE_DIR}/include/mrcap/Optimizer.h
${mrcap_SOURCE_DIR}/include/mrcap/FactorGraph.h
${mrcap_SOURCE_DIR}/include/mrcap/ROSCommunications.h
${mrcap_SOURCE_DIR}/include/mrcap/Metrics.h
${mrcap_SOURCE_DIR}/include/mrcap/MotionModelArc.h
${mrcap_SOURCE_DIR}/include/mrcap/SDF.h
${mrcap_SOURCE_DIR}/include/mrcap/DataTypes/RobotData.h
${mrcap_SOURCE_DIR}/include/mrcap/DataTypes/PositionPreset.h
${mrcap_SOURCE_DIR}/include/mrcap/DataTypes/CovarianceInfo.h
)
# find_package(Python3 REQUIRED COMPONENTS Development)
# include_directories(${Python3_INCLUDE_DIRS})

find_package(GTSAM REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs)
find_package(nav_msgs)
find_package(visualization_msgs)
find_package(NLopt REQUIRED)


message(STATUS "************** \n Package Statuses \n ____________________")
if(rclcpp_FOUND)
  message(STATUS "RCLCPP Found")
  include_directories(${rclcpp_INCLUDE_DIR})
  link_directories(${rclcpp_LIBRARY_DIRS})
endif()



if(geometry_msgs_FOUND)
  message(STATUS "geometry_msgs Found")
  include_directories(${geometry_msgs_INCLUDE_DIR})
  link_directories(${geometry_msgs_LIBRARY_DIRS})
endif()


if(ament_cmake_FOUND)
  message(STATUS "ament_cmake Found")
  include_directories(${ament_cmake_INCLUDE_DIR})
  link_directories(${ament_cmake_LIBRARY_DIRS})
endif()

if(gazebo_msgs_FOUND)
  message(STATUS "gazebo_msgs Found")
  include_directories(${gazebo_msgs_INCLUDE_DIR})
  link_directories(${gazebo_msgs_LIBRARY_DIRS})
endif()

if(nav_msgs_FOUND)
  message(STATUS "nav_msgs Found")
  include_directories(${nav_msgs_INCLUDE_DIR})
  link_directories(${nav_msgs_LIBRARY_DIRS})
endif()

if(visualization_msgs_FOUND)
  message(STATUS "visualization_msgs Found")
  include_directories(${visualization_msgs_INCLUDE_DIR})
  link_directories(${visualization_msgs_LIBRARY_DIRS})
endif()

if(NLopt_FOUND)
  message(STATUS "NLopt Found")
  # include_directories(${NLopt_INCLUDE_DIR})
  # link_directories(${NLopt_LIBRARY_DIRS})  

endif()

if(GTSAM_FOUND)
  message(STATUS "GTSAM Found")
  include_directories(${GTSAM_INCLUDE_DIR})
  link_directories(${GTSAM_LIBRARY_DIRS})
  # add_executable(mrcap_src main.cpp ${SRC_HEADER_LIST})
  # target_include_directories(mrcap_src PUBLIC ../include)
  # target_link_libraries (mrcap_src LINK_PUBLIC imgui implot gtsam ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES} ${gazebo_msgs_LIBRARIES}  ${nav_msgs_LIBRARIES} ${Python3_LIBRARIES})

  add_library(mrcap_lib STATIC ${SRC_HEADER_LIST})
  target_include_directories(mrcap_lib PUBLIC ../include)
  target_link_libraries(mrcap_lib PUBLIC NLopt::nlopt imgui implot gtsam ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES} ${gazebo_msgs_LIBRARIES} ${nav_msgs_LIBRARIES})
  add_executable(mrcap_src main.cpp)
  target_link_libraries(mrcap_src PRIVATE mrcap_lib)
  
else()
  message(WARNING "GTSAM is NOT found. GTSAM will not be used")
endif()


message(STATUS "  GTSAM include dir      Yes (${GTSAM_INCLUDE_DIR})")
message(STATUS "  GTSAM library dir        Yes (${GTSAM_LIBRARY_DIRS})")
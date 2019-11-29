cmake_minimum_required(VERSION 2.8.8)
project(fa_harris)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native -g" )

# Add -O0 to remove optimizations when using gcc
# IF(CMAKE_COMPILER_IS_GNUCC)
#     set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
#     set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g")
# ENDIF(CMAKE_COMPILER_IS_GNUCC)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dvs_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS 
    ${catkin_INCLUDE_DIRS}
)

include_directories(
  include/fa_harris/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
message("Opencv lib : ${OpenCV_INCLUDE_DIRS}")
set(LIBS ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})

list(APPEND HEADER_FILES
  include/fa_harris/include/fa_harris_detector.h
)

list(APPEND SOURCE_FILES
  include/fa_harris/src/fa_harris_detector.cc
)

add_executable(fa_harris ${SOURCE_FILES} src/fa_harris.cc)
target_link_libraries(fa_harris ${LIBS})

add_executable(ground_truth_klt ${SOURCE_FILES} src/evaluation/ground_truth_klt.cc)
target_link_libraries(ground_truth_klt ${LIBS})

add_executable(true_rate ${SOURCE_FILES} src/evaluation/true_rate.cc)
target_link_libraries(true_rate ${LIBS})
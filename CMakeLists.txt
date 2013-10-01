cmake_minimum_required(VERSION 2.8)
project(circle-detector)

SET(CMAKE_BUILD_TYPE debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O4 -march=native -Wfatal-errors")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake-configs")
option(ENABLE_3D "Enable 3D viewer support (requires PointClound library)" OFF)
option(ENABLE_MAVCONN "Enable MAVCONN support for running localization system as a service" OFF)

set(whycon_srcs circle_detector.cpp many_circle_detector.cpp localization_system.cpp)
set(REQUIRED_BOOST_COMPONENTS program_options)

## Check dependencies
if (ENABLE_3D)
  find_package(PCL QUIET COMPONENTS common io visualization)
  if (NOT PCL_FOUND)
  message(FATAL_ERROR "PointCloud library not found, not possible to enable 3D view")
  else()
  set(PCL_LIBS ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
  set(whycon_srcs ${whycon_srcs} localization_viewer.cpp)
  include_directories(${PCL_INCLUDE_DIRS})
  set(REQUIRED_BOOST_COMPONENTS ${REQUIRED_BOOST_COMPONENTS} thread system)
  endif()
endif()

if (ENABLE_MAVCONN)
  find_package(MAVLINK QUIET)
  find_package(MAVCONN QUIET)
  include_directories(${MAVCONN_INCLUDE_DIR} ${MAVLINK_INCLUDE_DIR} ${MAVLINK_INCLUDE_DIR}/..)
  set(LIBS ${LIBS} mavconn_lcm)
  set(whycon_srcs ${whycon_srcs} localization_service.cpp)
endif()

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(Boost COMPONENTS ${REQUIRED_BOOST_COMPONENTS} REQUIRED)
set(LIBS ${LIBS} ${OpenCV_LIBS} ${GSL_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBS})

# create config file
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/config.h.cmake ${CMAKE_SOURCE_DIR}/config.h)

# generate whycon shared library
add_library(whycon SHARED ${whycon_srcs})

# generate extra executables
add_executable(localization-system main.cpp)
target_link_libraries(localization-system whycon ${LIBS})

SET(camera_calibrator_sources camera_calibrator.cpp)
add_executable(camera_calibrator ${camera_calibrator_sources})
target_link_libraries(camera_calibrator ${OpenCV_LIBS})

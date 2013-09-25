cmake_minimum_required(VERSION 2.8)
project(circle-detector)

SET(CMAKE_BUILD_TYPE debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O4 -march=native -Wfatal-errors")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
option(ENABLE_3D "Enable 3D viewer support (requires PointClound library)" OFF)

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

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(Boost COMPONENTS ${REQUIRED_BOOST_COMPONENTS} REQUIRED)
set(LIBS ${OpenCV_LIBS} ${TBB_LIBRARIES} ${GSL_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBS})

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/config.h.cmake ${CMAKE_SOURCE_DIR}/config.h)


add_library(whycon SHARED ${whycon_srcs})

add_executable(localization-system main.cpp)
target_link_libraries(localization-system whycon ${LIBS})

SET(calibrator_sources calibrator.cpp)
add_executable(calibrator ${calibrator_sources})
target_link_libraries(calibrator ${OpenCV_LIBS})

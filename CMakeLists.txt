cmake_minimum_required(VERSION 2.8)
project(circle-detector)

SET(CMAKE_BUILD_TYPE debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O4 -march=native -Wfatal-errors")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake-configs")
option(ENABLE_MAVCONN "Enable MAVCONN support for running localization system as a service" OFF)
option(ENABLE_VIEWER "Enable 3D viewer support (requires PointClound library and MAVCONN enabled)" OFF)
option(ENABLE_FULL_UNDISTORT "Undistort the whole frame" OFF)
option(ENABLE_RANDOMIZED_THRESHOLD "Use rand() instead of binary-like search for threshold" OFF)

set(whycon_srcs circle_detector.cpp many_circle_detector.cpp localization_system.cpp)

## Check dependencies
find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(Boost COMPONENTS program_options thread system REQUIRED)

if (ENABLE_MAVCONN)
  find_package(MAVLINK QUIET)
  find_package(MAVCONN QUIET)
  include_directories(${MAVCONN_INCLUDE_DIR} ${MAVLINK_INCLUDE_DIR} ${MAVLINK_INCLUDE_DIR}/..)
  set(MAVCONN_LIBS mavconn_lcm lcm)  
  set(whycon_srcs ${whycon_srcs} localization_service.cpp)

  find_package(PythonInterp 2 REQUIRED)
  ADD_CUSTOM_COMMAND(
     OUTPUT ${CMAKE_CURRENT_LIST_DIR}/mavlink/whycon/mavlink.h
     COMMAND ${PYTHON_EXECUTABLE} ${MAVLINK_MAVGEN} --lang=C --output=mavlink whycon.xml --wire-protocol=1.0
     DEPENDS ${CMAKE_CURRENT_LIST_DIR}/whycon.xml
     WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  )
  add_custom_target(mavlink_headers ALL DEPENDS ${CMAKE_CURRENT_LIST_DIR}/mavlink/whycon/mavlink.h)

  if (ENABLE_VIEWER)
    add_subdirectory(viewer)
  endif()
endif()

# create config file
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/config.h.cmake ${CMAKE_SOURCE_DIR}/config.h)

# generate whycon shared library
add_library(whycon SHARED ${whycon_srcs})

# generate extra executables
add_executable(localization-system main.cpp)
target_link_libraries(localization-system whycon ${OpenCV_LIBS} ${GSL_LIBRARIES} ${Boost_LIBRARIES} ${MAVCONN_LIBS})

SET(camera_calibrator_sources camera_calibrator.cpp)
add_executable(camera_calibrator ${camera_calibrator_sources})
target_link_libraries(camera_calibrator ${OpenCV_LIBS})


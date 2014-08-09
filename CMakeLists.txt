cmake_minimum_required(VERSION 2.8)
project(whycon)

#SET(CMAKE_BUILD_TYPE debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O4 -march=native -Wfatal-errors")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake-configs")
option(ENABLE_FULL_UNDISTORT "Undistort the whole frame" OFF)
option(ENABLE_RANDOMIZED_THRESHOLD "Use rand() instead of binary-like search for threshold" OFF)

set(whycon_srcs circle_detector.cpp many_circle_detector.cpp localization_system.cpp)

## Check dependencies
find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(Boost COMPONENTS program_options thread system REQUIRED)

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

# install targets
install(TARGETS whycon DESTINATION lib)
install(TARGETS localization-system camera_calibrator DESTINATION bin)
install(FILES circle_detector.h config.h localization_system.h many_circle_detector.h DESTINATION include/whycon)
install(FILES cmake-configs/FindWhyCon.cmake cmake-configs/FindGSL.cmake DESTINATION share/cmake/Modules)
install(FILES package.xml DESTINATION share/whycon)

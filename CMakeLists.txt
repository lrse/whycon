cmake_minimum_required(VERSION 2.8)
project(circle-detector)

SET(CMAKE_BUILD_TYPE debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O4 -march=native -Wfatal-errors")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}") 

#CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/config.h.cmake ${CMAKE_SOURCE_DIR}/config.h)

#find_package(TBB)
find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
#find_package(Boost COMPONENTS thread filesystem system REQUIRED)

#set(LIBS ${OpenCV_LIBS} ${Boost_LIBRARIES} ${TBB_LIBRARIES})
set(LIBS ${OpenCV_LIBS} ${TBB_LIBRARIES} ${GSL_LIBRARIES})

SET(detector_sources test-detector.cpp circle_detector.cpp many_circle_detector.cpp)
add_executable(circle-detector ${detector_sources})
target_link_libraries(circle-detector ${LIBS})

SET(system_sources test-system.cpp circle_detector.cpp many_circle_detector.cpp localization_system.cpp)
add_executable(localization-system ${system_sources})
target_link_libraries(localization-system ${LIBS})

SET(calibrator_sources calibrator.cpp)
add_executable(calibrator ${calibrator_sources})
target_link_libraries(calibrator ${OpenCV_LIBS})

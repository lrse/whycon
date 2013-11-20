set(whycon_srcs circle_detector.cpp many_circle_detector.cpp localization_system.cpp)

find_package(PCL QUIET COMPONENTS common io visualization)
if (NOT PCL_FOUND)
  message(FATAL_ERROR "PointCloud library not found, not possible to enable 3D viewer")
endif()

set(MAVCONN_LIBS ${MAVCONN_LIBS} PARENT_SCOPE)
set(PCL_LIBS ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
include_directories(${PCL_INCLUDE_DIRS})
add_executable(localization-viewer main.cpp localization_client.cpp localization_viewer.cpp)
target_link_libraries(localization-viewer ${Boost_LIBRARIES} ${PCL_LIBS} ${MAVCONN_LIBS})

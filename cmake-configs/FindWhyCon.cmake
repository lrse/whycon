find_path(WHYCON_INCLUDE_DIRS whycon/localization_system.h PATH_SUFFIXES whycon HINTS ${CMAKE_CURRENT_LIST_DIR}/../../../include)
find_library(WHYCON_LIBRARIES NAMES whycon HINTS ${CMAKE_CURRENT_LIST_DIR}/../../../lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(whycon DEFAULT_MSG WHYCON_LIBRARIES WHYCON_INCLUDE_DIRS)

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(Boost COMPONENTS program_options thread system REQUIRED)

set(WHYCON_LIBRARIES ${WHYCON_LIBRARIES} ${OpenCV_LIBRARIES} ${GSL_LIBRARIES} ${Boost_LIBRARIES})
set(WHYCON_INCLUDE_DIRS ${WHYCON_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${GSL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

mark_as_advanced(WHYCON_LIBRARIES WHYCON_INCLUDE_DIRS)

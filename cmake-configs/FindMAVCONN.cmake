INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(MAVCONN_IncludeSearchPaths
  /usr/include/mavconn/
  /usr/local/include/mavconn/
  /opt/local/include/mavconn/
)

SET(MAVCONN_LibrarySearchPaths
  /usr/lib/mavconn/
  /usr/local/lib/mavconn/
  /opt/local/lib/mavconn/
)

FIND_PATH(MAVCONN_INCLUDE_DIR mavconn.h
  PATHS ${MAVCONN_IncludeSearchPaths}
)
FIND_LIBRARY(MAVCONN_LIBRARY_OPTIMIZED
  NAMES mavconn_lcm
  PATHS ${MAVCONN_LibrarySearchPaths}
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MAVCONN "Could NOT find mavconn library (MAVCONN)"
  MAVCONN_LIBRARY_OPTIMIZED
  MAVCONN_INCLUDE_DIR
)


# Collect optimized and debug libraries
HANDLE_LIBRARY_TYPES(MAVCONN_LCM)

MARK_AS_ADVANCED(
  MAVCONN_INCLUDE_DIR
  MAVCONN_LIBRARY_OPTIMIZED
)

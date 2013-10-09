INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(MAVLINK_IncludeSearchPaths
  /usr/include/mavlink/v1.0/pixhawk
  /usr/include/mavlink/v1.0
  /usr/local/include/mavlink/v1.0/pixhawk
  /usr/local/include/mavlink/v1.0
  )

SET(MAVLINK_MavgenSearchPaths
  /usr/share/pyshared/pymavlink/generator
  /usr/local/share/pyshared/pymavlink/generator
)

FIND_PATH(MAVLINK_INCLUDE_DIR
  NAMES mavlink.h
  PATHS ${MAVLINK_IncludeSearchPaths}
)

FIND_PATH(MAVLINK_TYPES_INCLUDE_DIR
  NAMES mavlink_types.h
  PATHS ${MAVLINK_IncludeSearchPaths}
)

FIND_FILE(MAVLINK_MAVGEN
  NAMES mavgen.py
  PATHS ${MAVLINK_MavgenSearchPaths}
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MAVLINK "Could NOT find MAVLink protocol (mavlink.h)"
  MAVLINK_INCLUDE_DIR
  MAVLINK_TYPES_INCLUDE_DIR
  MAVLINK_MAVGEN
)

MARK_AS_ADVANCED(
  MAVLINK_INCLUDE_DIR
  MAVLINK_TYPES_INCLUDE_DIR
  MAVLINK_MAVGEN
)

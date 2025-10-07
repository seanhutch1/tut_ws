#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "iar_dijkstra_planner::iar_dijkstra_planner" for configuration ""
set_property(TARGET iar_dijkstra_planner::iar_dijkstra_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(iar_dijkstra_planner::iar_dijkstra_planner PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libiar_dijkstra_planner.so"
  IMPORTED_SONAME_NOCONFIG "libiar_dijkstra_planner.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS iar_dijkstra_planner::iar_dijkstra_planner )
list(APPEND _IMPORT_CHECK_FILES_FOR_iar_dijkstra_planner::iar_dijkstra_planner "${_IMPORT_PREFIX}/lib/libiar_dijkstra_planner.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

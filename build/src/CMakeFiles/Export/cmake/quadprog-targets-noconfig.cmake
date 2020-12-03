#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "quadprog" for configuration ""
set_property(TARGET quadprog APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(quadprog PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libquadprog.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS quadprog )
list(APPEND _IMPORT_CHECK_FILES_FOR_quadprog "${_IMPORT_PREFIX}/lib/libquadprog.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

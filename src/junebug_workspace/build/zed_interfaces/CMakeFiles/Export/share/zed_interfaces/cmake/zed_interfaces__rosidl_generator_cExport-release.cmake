#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "zed_interfaces::zed_interfaces__rosidl_generator_c" for configuration "Release"
set_property(TARGET zed_interfaces::zed_interfaces__rosidl_generator_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(zed_interfaces::zed_interfaces__rosidl_generator_c PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libzed_interfaces__rosidl_generator_c.so"
  IMPORTED_SONAME_RELEASE "libzed_interfaces__rosidl_generator_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS zed_interfaces::zed_interfaces__rosidl_generator_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_zed_interfaces::zed_interfaces__rosidl_generator_c "${_IMPORT_PREFIX}/lib/libzed_interfaces__rosidl_generator_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

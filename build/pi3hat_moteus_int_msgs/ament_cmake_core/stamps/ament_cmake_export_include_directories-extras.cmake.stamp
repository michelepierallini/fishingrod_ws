# generated from ament_cmake_export_include_directories/cmake/ament_cmake_export_include_directories-extras.cmake.in

set(_exported_include_dirs "${pi3hat_moteus_int_msgs_DIR}/../../../include/pi3hat_moteus_int_msgs")

# append include directories to pi3hat_moteus_int_msgs_INCLUDE_DIRS
# warn about not existing paths
if(NOT _exported_include_dirs STREQUAL "")
  find_package(ament_cmake_core QUIET REQUIRED)
  foreach(_exported_include_dir ${_exported_include_dirs})
    if(NOT IS_DIRECTORY "${_exported_include_dir}")
      message(WARNING "Package 'pi3hat_moteus_int_msgs' exports the include directory '${_exported_include_dir}' which doesn't exist")
    endif()
    normalize_path(_exported_include_dir "${_exported_include_dir}")
    list(APPEND pi3hat_moteus_int_msgs_INCLUDE_DIRS "${_exported_include_dir}")
  endforeach()
endif()

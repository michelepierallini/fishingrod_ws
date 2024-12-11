# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fishingrod_gazebo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fishingrod_gazebo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fishingrod_gazebo_FOUND FALSE)
  elseif(NOT fishingrod_gazebo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fishingrod_gazebo_FOUND FALSE)
  endif()
  return()
endif()
set(_fishingrod_gazebo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fishingrod_gazebo_FIND_QUIETLY)
  message(STATUS "Found fishingrod_gazebo: 0.0.0 (${fishingrod_gazebo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fishingrod_gazebo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fishingrod_gazebo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fishingrod_gazebo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fishingrod_gazebo_DIR}/${_extra}")
endforeach()

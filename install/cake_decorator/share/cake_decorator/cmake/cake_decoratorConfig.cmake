# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cake_decorator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cake_decorator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cake_decorator_FOUND FALSE)
  elseif(NOT cake_decorator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cake_decorator_FOUND FALSE)
  endif()
  return()
endif()
set(_cake_decorator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cake_decorator_FIND_QUIETLY)
  message(STATUS "Found cake_decorator: 0.0.0 (${cake_decorator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cake_decorator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cake_decorator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cake_decorator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cake_decorator_DIR}/${_extra}")
endforeach()

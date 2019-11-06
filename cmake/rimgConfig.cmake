# =============================================================================
# The rimg CMake configuration file.
#
#           ** File generated automatically, DO NOT MODIFY! ***

# To use from an external project, in your project's CMakeLists.txt add:
#   FIND_PACKAGE( rimg REQUIRED)
#   INCLUDE_DIRECTORIES( rimg ${rimg_INCLUDE_DIRS})
#   LINK_DIRECTORIES( ${rimg_LIBRARY_DIR})
#   TARGET_LINK_LIBRARIES( MY_TARGET_NAME ${rimg_LIBRARIES})
#
# This module defines the following variables:
#   - rimg_FOUND         : True if rimg is found.
#   - rimg_ROOT_DIR      : The root directory where rimg is installed.
#   - rimg_INCLUDE_DIRS  : The rimg include directories.
#   - rimg_LIBRARY_DIR   : The rimg library directory.
#   - rimg_LIBRARIES     : The rimg imported libraries to link to.
#
# =============================================================================

get_filename_component( rimg_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component( rimg_ROOT_DIR  "${rimg_CMAKE_DIR}"           PATH)

set( rimg_INCLUDE_DIRS "${rimg_ROOT_DIR}/../include" CACHE PATH "The rimg include directories.")
set( rimg_LIBRARY_DIR  "${rimg_ROOT_DIR}"            CACHE PATH "The rimg library directory.")

include( "${CMAKE_CURRENT_LIST_DIR}/Macros.cmake")
get_library_suffix( _lsuff)
set( _hints rimg${_lsuff} librimg${_lsuff})
find_library( rimg_LIBRARIES NAMES ${_hints} PATHS "${rimg_LIBRARY_DIR}/static" "${rimg_LIBRARY_DIR}")
set( rimg_LIBRARIES     ${rimg_LIBRARIES}         CACHE FILEPATH "The rimg imported libraries to link to.")

# handle QUIETLY and REQUIRED args and set rimg_FOUND to TRUE if all listed variables are TRUE
include( "${CMAKE_ROOT}/Modules/FindPackageHandleStandardArgs.cmake")
find_package_handle_standard_args( rimg rimg_FOUND rimg_LIBRARIES rimg_INCLUDE_DIRS)

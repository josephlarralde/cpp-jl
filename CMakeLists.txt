cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(jl)
# option(JL_INCLUDE_SOURCE_ONLY "Create a simple jl target for including headers" OFF)

# if(JL_INCLUDE_SOURCE_ONLY)
#   set(JL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
#   set(JL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "")
#   include(cmake/jl.sources.cmake)
#   return()
# endif()

include(cmake/jl.cmake)

# Only build examples and tests if we are building this repo directly,
# not if it is add_subdirectory'd
# if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
#   include(jl.examples)
#   include(jl.tests)
# endif()
cmake_minimum_required(VERSION 3.5)

project(libMultiRobotPlanning)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# check if Doxygen is installed
find_package(Doxygen)

if (DOXYGEN_FOUND)
  # set input and output files
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  # request to configure the file
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  message("Doxygen build started")

  # note the option ALL which allows to build the docs together with the application
  add_custom_target( docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

# clang-tidy target (linter & static code analysis)
add_custom_target(clang-tidy
  COMMAND CMAKE_EXPORT_COMPILE_COMMANDS=ON run-clang-tidy ${CMAKE_CURRENT_SOURCE_DIR})

# clang-format
set(ALL_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/a_star.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/a_star_epsilon.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/assignment.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/cbs.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/cbs_ta.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/ecbs.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/ecbs_ta.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/neighbor.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/next_best_assignment.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/planresult.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libMultiRobotPlanning/sipp.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/a_star.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/a_star_epsilon.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/assignment.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/cbs.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/cbs_ta.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/ecbs.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/ecbs_ta.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/mapf_prioritized_sipp.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/next_best_assignment.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/shortest_path_heuristic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/shortest_path_heuristic.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/sipp.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/timer.hpp
)

add_custom_target(clang-format
  COMMAND clang-format -i ${ALL_SOURCE_FILES}
)

# tests
add_custom_target(run-test
  COMMAND python3 -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_custom_target(everything
  DEPENDS clang-format clang-tidy docs run-test
)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Creates compile database used by clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
)

# Examples

## a_star
add_executable(a_star
  example/a_star.cpp
)
target_link_libraries(a_star
  ${Boost_LIBRARIES}
)

## a_star_epsilon
add_executable(a_star_epsilon
  example/a_star_epsilon.cpp
)
target_link_libraries(a_star_epsilon
  ${Boost_LIBRARIES}
)

## assignment
add_executable(assignment
  example/assignment.cpp
)
target_link_libraries(assignment
  ${Boost_LIBRARIES}
)

## cbs_roadmap
add_executable(cbs_roadmap
  example/cbs_roadmap.cpp
)
target_link_libraries(cbs_roadmap
  ${Boost_LIBRARIES}
  yaml-cpp
)

## cbs
add_executable(cbs
  example/cbs.cpp
)
target_link_libraries(cbs
  ${Boost_LIBRARIES}
  yaml-cpp
)

## cbs_ta
add_executable(cbs_ta
  example/cbs_ta.cpp
)
target_link_libraries(cbs_ta
  ${Boost_LIBRARIES}
  yaml-cpp
)

## ecbs
add_executable(ecbs
  example/ecbs.cpp
)
target_link_libraries(ecbs
  ${Boost_LIBRARIES}
  yaml-cpp
)

## ecbs_ta
add_executable(ecbs_ta
  example/ecbs_ta.cpp
)
target_link_libraries(ecbs_ta
  ${Boost_LIBRARIES}
  yaml-cpp
)

## next_best_assignment
add_executable(next_best_assignment
  example/next_best_assignment.cpp
)
target_link_libraries(next_best_assignment
  ${Boost_LIBRARIES}
  yaml-cpp
)

## sipp
add_executable(sipp
  example/sipp.cpp
)
target_link_libraries(sipp
  ${Boost_LIBRARIES}
  yaml-cpp
)

## mapf_prioritized_sipp
add_executable(mapf_prioritized_sipp
  example/mapf_prioritized_sipp.cpp
)
target_link_libraries(mapf_prioritized_sipp
  ${Boost_LIBRARIES}
  yaml-cpp
)

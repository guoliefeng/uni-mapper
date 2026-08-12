
add_subdirectory(hmm_mos)
add_subdirectory(otd)

set(OPEN_LMM_BUILD_DUFOMAP_DEFAULT ON)
if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1" AND
   CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
   CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10")
  set(OPEN_LMM_BUILD_DUFOMAP_DEFAULT OFF)
endif()
option(OPEN_LMM_BUILD_DUFOMAP
  "Build DUFOMap (requires a compiler with C++20 concepts support)"
  ${OPEN_LMM_BUILD_DUFOMAP_DEFAULT})

set(online_remover_libraries hmm_mos otd)
if(OPEN_LMM_BUILD_DUFOMAP)
  add_subdirectory(dufomap)
  list(APPEND online_remover_libraries dufomap)
else()
  message(WARNING
    "DUFOMap is disabled because this ROS1 compiler lacks C++20 concepts. "
    "Use GCC 10+ and -DOPEN_LMM_BUILD_DUFOMAP=ON to enable it.")
endif()

add_library(remover_online INTERFACE)
target_link_libraries(remover_online INTERFACE
  ${online_remover_libraries}
)

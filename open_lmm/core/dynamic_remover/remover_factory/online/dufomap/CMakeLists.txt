message(STATUS "================  BUILDING DUFOMAP  ======================")

#! sudo apt install liblzf-dev
include(FetchContent)
FetchContent_Declare(
    ufomap
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufomap.git
    GIT_TAG        fcb20f445d257280c039c07ab756c9d5b37ebd78
    PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/ufomap.patch UPDATE_DISCONNECTED 1
)
FetchContent_MakeAvailable(ufomap)

add_library(dufomap SHARED)
target_sources(dufomap PRIVATE
  dufomap.cpp
  params.cpp
)
target_link_libraries(dufomap PUBLIC
  UFO::Map
  Eigen3::Eigen
  PCL::PCL
  open_lmm_utils
)
openlmm_set_global_target_properties(dufomap)
install(TARGETS dufomap
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(create_dufomap SHARED)
target_sources(create_dufomap PRIVATE 
create_dufomap.cpp
)
target_link_libraries(create_dufomap
dufomap
)
openlmm_set_create_target_properties(create_dufomap)

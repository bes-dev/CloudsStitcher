##################################
# cstitcher library
##################################

include_directories(${CSTITCHER_DEPENDENCY_DIRS})

##################################
# Sources

file(GLOB_RECURSE sources_cpp src/*.cpp)
file(GLOB_RECURSE sources_h include/*.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
##################################

##################################
# Target

add_library(cstitcher ${sources_cpp} ${sources_h})
target_link_libraries(cstitcher ${CSTITCHER_DEPENDENCY_LIBS})

##################################

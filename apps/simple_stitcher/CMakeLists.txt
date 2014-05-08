##################################
# cstitcher simple app
##################################

include_directories(${CSTITCHER_DEPENDENCY_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/cstitcher/include)

##################################
# Sources

file(GLOB_RECURSE sources_cpp *.cpp)
file(GLOB_RECURSE sources_h *.cpp)
##################################

##################################
# Target

add_executable(simple_stitcher ${sources_cpp} ${sources_h})
target_link_libraries(simple_stitcher cstitcher ${CSTITCHER_DEPENDENCY_LIBS})
##################################

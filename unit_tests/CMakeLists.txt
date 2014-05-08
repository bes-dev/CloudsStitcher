##################################
# cstitcher_unittest
##################################

include_directories(${CSTITCHER_DEPENDENCY_DIRS})

##################################
# Sources

file(GLOB_RECURSE sources_cpp *.cpp)
file(GLOB_RECURSE sources_cc *.cc)
file(GLOB_RECURSE sources_h *.h)

include_directories ("${PROJECT_SOURCE_DIR}/cstitcher/include")
##################################


##################################
# Target
add_executable(cstitcher_unittest ${sources_cc} ${sources_cpp} ${sources_h})
target_link_libraries(cstitcher_unittest cstitcher ${CSTITCHER_DEPENDENCY_LIBS})

##################################

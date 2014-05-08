#############################################
# CloudStitcher project
#############################################
cmake_minimum_required(VERSION 2.8)

project(CloudStitcher)
set(CloudStitcher_VERSION_MAJOR 0)
set(CloudStitcher_VERSION_MINOR 1)

#############################################
# Dependencies

#OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(SEND_ERROR "Faild to find Opencv")
    return()
else()
    set(CSTITCHER_DEPENDENCY_DIRS ${OpenCV_INCLUDE_DIRS} ${CSTITCHER_DEPENDENCY_DIRS})
    set(CSTITCHER_DEPENDENCY_LIBS ${OpenCV_LIBS} ${CSTITCHER_DEPENDENCY_LIBS})
endif()
#############################################


#############################################
#General compiler flags

add_definitions(-DDEBUG_MODE) #This shows all messages from LOG()
add_definitions(-Wall) #Enable all warnings always
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -mtune=corei7 -march=corei7")

#############################################

add_subdirectory(cstitcher)
add_subdirectory(apps)
add_subdirectory(unit_tests)

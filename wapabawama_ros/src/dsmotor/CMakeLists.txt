cmake_minimum_required (VERSION 3.14)

# projectname is the same as the main-executable
project(dsMotor)
set (CMAKE_CXX_STANDARD 20)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

file( GLOB SRCS src/*.c src/*.cpp src/*.cc )

# find_package( OpenCV REQUIRED )
# include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories( include )

add_executable( ${PROJECT_NAME} ${SRCS} )

# target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

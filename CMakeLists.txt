cmake_minimum_required(VERSION 3.0)

project(getTau)

#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")

FIND_PACKAGE(Boost 1.65 COMPONENTS filesystem system iostreams REQUIRED)
include_directories(${CMAKE_HOME_DIRECTORY})

file(GLOB source *.cpp)
file(GLOB header *.h)
add_executable(${PROJECT_NAME} ${source} ${header})

target_link_libraries( ${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES} )

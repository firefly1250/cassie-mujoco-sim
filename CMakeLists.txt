cmake_minimum_required(VERSION 3.16)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 20) # C++20...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-long-long -pedantic")
set(JSON_BuildTests OFF CACHE INTERNAL "")

project(cassiemujoco)

add_library(cassiemujoco
  src/cassiemujoco.c 
  src/udp.c
)

target_include_directories(cassiemujoco PUBLIC 
  ~/.mujoco/mujoco200_linux/include
  include 
)

target_link_libraries(cassiemujoco
  ${CMAKE_CURRENT_SOURCE_DIR}/src/libagilitycassie.a
  ${CMAKE_DL_LIBS}
  m
)

add_subdirectory(example)

cmake_minimum_required(VERSION 3.12)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
IF(MSVC)
  set(CMAKE_CXX_FLAGS "/W4 -O2 \
    /wd4100 /wd4458 /wd4577 /wd4267 /wd4244 /wd4505 /wd4838 \
    /wd4800 /wd4996 /wd4530 /wd4245 /wd4505 /wd4505 /wd4456 ")
ELSE()
  set(CMAKE_CXX_FLAGS "-Wall -Wno-deprecated-declarations -g")
ENDIF()

project(task8)

################################

# glfw
SET(GLFW_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/GLFW_Lib) # specify candidate for glfw location
include(${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/FindGLFW.cmake) # call script to find glfw

# opengl
find_package(OpenGL REQUIRED)

# eigen
FILE(GLOB EIGEN3_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/eigen*) # find downloaded eigen directories in 3rd_party
SET(EIGEN3_ROOT
  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/Eigen_Lib/include # installed eigen is higher priority
  ${EIGEN3_ROOT} ) # specify candidate for eigen location
include(${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/FindEigen3.cmake) # call script to find eigen3
#find_package(Eigen3 REQUIRED) # this works only for installation from package manager
message(STATUS "eigen3 locatoin: ${EIGEN3_INCLUDE_DIR}")

# delfem2
add_definitions(-DDFM2_HEADER_ONLY=ON)
set(DELFEM2_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/delfem2/include")

################################

include_directories(
  ${OPENGL_INCLUDE_DIR}
  ${GLFW_INCLUDE_DIR}
  ${DELFEM2_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
  main.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OPENGL_LIBRARY}
  ${GLFW_LIBRARIES}
)
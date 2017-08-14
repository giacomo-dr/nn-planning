cmake_minimum_required(VERSION 2.8)

PROJECT(MantaController)

# --------------- V-Rep includes directory ---------------
set(VREP_BASE "/Applications/V-REP_PRO_EDU_V3_4_0_Mac/programming")
include_directories("${VREP_BASE}/include")
include_directories("${VREP_BASE}/remoteApi")


# --------------- Eigen ---------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# --------------- BOOST ---------------
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.59.0 COMPONENTS regex)
include_directories(${Boost_INCLUDE_DIRS})


# --------------- OpenCV ---------------
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# --------------- Compiler flags ---------------
set(CMAKE_C_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255")
if(CMAKE_SYSTEM MATCHES "Linux-([3-9]\\.[0-9]|2\\.[4-9])\\.")
    set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -D__linux" )
else()
    set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -D__APPLE__" )
endif(CMAKE_SYSTEM MATCHES "Linux-([3-9]\\.[0-9]|2\\.[4-9])\\.")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wall -std=c++11 -g")


# --------------- Executable ---------------
set(VREP_SRCS "${VREP_BASE}/remoteApi/extApi.c" "${VREP_BASE}/remoteApi/extApiPlatform.c")
file(GLOB_RECURSE MANTA_SRCS "src/*.cpp")
list(REMOVE_ITEM MANTA_SRCS "${CMAKE_SOURCE_DIR}/src/executor-path.cpp")  # Compile only one main() at once
add_executable(mantacontroller ${MANTA_SRCS} ${VREP_SRCS})
target_link_libraries(mantacontroller ${Boost_LIBRARIES} ${OpenCV_LIBS})



cmake_minimum_required(VERSION 3.5)

PROJECT(NNPlanning)
set(CMAKE_CXX_STANDARD 11)

# --------------- antlr4 ---------------
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(ANTLR4CPP_JAR_LOCATION ${PROJECT_SOURCE_DIR}/thirdparty/antlr/antlr-4.7-complete.jar)
include(ExternalAntlr4Cpp)
include_directories(${ANTLR4CPP_INCLUDE_DIRS})
link_directories(${ANTLR4CPP_LIBS})
message(STATUS "Found antlr4cpp libs: ${ANTLR4CPP_LIBS} and includes: ${ANTLR4CPP_INCLUDE_DIRS}")
antlr4cpp_process_grammar(mantacontroller nnplanning
        ${CMAKE_CURRENT_SOURCE_DIR}/grammars/DOT.g4
        ${CMAKE_CURRENT_SOURCE_DIR}/grammars/emptylexer.g4)
include_directories(${antlr4cpp_include_dirs_nnplanning})


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
list(REMOVE_ITEM MANTA_SRCS "${CMAKE_SOURCE_DIR}/src/main-pathfollowing.cpp")  # Compile only one main() at once
list(REMOVE_ITEM MANTA_SRCS "${CMAKE_SOURCE_DIR}/src/main-reachtarget.cpp")  # Compile only one main() at once
list(REMOVE_ITEM MANTA_SRCS "${CMAKE_SOURCE_DIR}/src/main-planning.cpp")  # Compile only one main() at once
list(REMOVE_ITEM MANTA_SRCS "${CMAKE_SOURCE_DIR}/src/main-maptest.cpp")  # Compile only one main() at once
add_executable(mantacontroller "${CMAKE_SOURCE_DIR}/src/main-planning.cpp" ${MANTA_SRCS} ${VREP_SRCS} ${antlr4cpp_src_files_nnplanning})
add_dependencies(mantacontroller antlr4cpp antlr4cpp_generation_nnplanning)
target_link_libraries(mantacontroller ${Boost_LIBRARIES} ${OpenCV_LIBS} antlr4-runtime)

# --------------- Test ---------------
include_directories("${CMAKE_SOURCE_DIR}/src")
add_executable(testdrv "${CMAKE_SOURCE_DIR}/test/planning-test-driver.cpp" ${MANTA_SRCS} ${VREP_SRCS} ${antlr4cpp_src_files_nnplanning})
add_dependencies(testdrv antlr4cpp antlr4cpp_generation_nnplanning)
target_link_libraries(testdrv ${Boost_LIBRARIES} ${OpenCV_LIBS} antlr4-runtime)

# The following command is needed to find antlr4 dll
add_custom_command(TARGET mantacontroller POST_BUILD
    COMMAND install_name_tool -change libantlr4-runtime.4.7.dylib @executable_path/locals/antlr4cpp/lib/libantlr4-runtime.4.7.dylib mantacontroller
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Adjust path to libantlr4-runtime.4.7.dylib into mantacontroller executable"
)

add_custom_command(TARGET testdrv POST_BUILD
    COMMAND install_name_tool -change libantlr4-runtime.4.7.dylib @executable_path/locals/antlr4cpp/lib/libantlr4-runtime.4.7.dylib testdrv
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Adjust path to libantlr4-runtime.4.7.dylib into test executable"
)

project(pbs)

cmake_minimum_required (VERSION 3.7.1)

set(CMAKE_CXX_STANDARD 17)  # only useful when CMake version >= 3.1
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_LINKER  "/usr/bin/ld")
set(CMAKE_VERBOSE_MAKEFILE OFF)  # Show all commands and parameters

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_executable(pbs ${SOURCES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(pbs ${Boost_LIBRARIES})

# Find CPLEX
find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(pbs PUBLIC "-pthread")
endif()

include_directories("/opt/ibm/ILOG/CPLEX_Studio201/cplex/include")
include_directories("/opt/ibm/ILOG/CPLEX_Studio201/concert/include")
find_library(ILOCPLEX_LIBRARY ilocplex "/opt/ibm/ILOG/CPLEX_Studio201/cplex/lib/x86-64_linux/static_pic")
find_library(CPLEX_LIBRARY cplex "/opt/ibm/ILOG/CPLEX_Studio201/cplex/lib/x86-64_linux/static_pic")
find_library(CONCERT_LIBRARY concert "/opt/ibm/ILOG/CPLEX_Studio201/concert/lib/x86-64_linux/static_pic")

target_link_libraries(pbs ${ILOCPLEX_LIBRARY} -lpthread -ldl)
target_link_libraries(pbs ${CPLEX_LIBRARY} -lpthread -ldl)
target_link_libraries(pbs ${CONCERT_LIBRARY} -lpthread -ldl)
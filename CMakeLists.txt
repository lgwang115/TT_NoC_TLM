cmake_minimum_required(VERSION 3.16)
project(noc_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Try to locate SystemC
if(NOT SYSTEMC_HOME)
  set(SYSTEMC_HOME $ENV{SYSTEMC_HOME})
endif()

if(SYSTEMC_HOME)
  set(SYSTEMC_INCLUDE_DIR "${SYSTEMC_HOME}/include")
  if(EXISTS "${SYSTEMC_HOME}/lib-linux64")
    set(SYSTEMC_LIB_DIR "${SYSTEMC_HOME}/lib-linux64")
  elseif(EXISTS "${SYSTEMC_HOME}/lib64")
    set(SYSTEMC_LIB_DIR "${SYSTEMC_HOME}/lib64")
  else()
    set(SYSTEMC_LIB_DIR "${SYSTEMC_HOME}/lib")
  endif()
endif()

find_path(SYSTEMC_INCLUDE_DIR systemc HINTS ${SYSTEMC_INCLUDE_DIR})
find_library(SYSTEMC_LIBRARY systemc HINTS ${SYSTEMC_LIB_DIR})

if(NOT SYSTEMC_INCLUDE_DIR OR NOT SYSTEMC_LIBRARY)
  message(FATAL_ERROR "SystemC not found. Set SYSTEMC_HOME or SYSTEMC_INCLUDE_DIR/SYSTEMC_LIB_DIR")
endif()

add_executable(noc_sim
  src/main.cpp
  src/noc_link.cpp
  src/noc_router.cpp
  src/noc_mesh.cpp
  src/noc_niu.cpp
  src/noc_translation.cpp
  src/riscv_core.cpp
  src/tensix_tile.cpp
  src/traffic_gen.cpp
)

target_include_directories(noc_sim PRIVATE ${SYSTEMC_INCLUDE_DIR} src)

target_link_libraries(noc_sim PRIVATE ${SYSTEMC_LIBRARY})

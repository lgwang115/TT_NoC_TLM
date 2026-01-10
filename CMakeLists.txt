cmake_minimum_required(VERSION 3.16)
project(noc_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to build tests
option(BUILD_TESTS "Build unit tests" ON)

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

# Common source files (excluding main.cpp for reuse in tests)
set(NOC_SIM_SOURCES
  src/noc_link.cpp
  src/noc_router.cpp
  src/noc_mesh.cpp
  src/noc_niu.cpp
  src/noc_overlay.cpp
  src/noc_translation.cpp
  src/riscv_core.cpp
  src/tensix_tile.cpp
  src/traffic_gen.cpp
)

# Main simulation executable
add_executable(noc_sim
  src/main.cpp
  ${NOC_SIM_SOURCES}
)

target_include_directories(noc_sim PRIVATE ${SYSTEMC_INCLUDE_DIR} src)
target_link_libraries(noc_sim PRIVATE ${SYSTEMC_LIBRARY})

# Testing support
if(BUILD_TESTS)
  # Try to find GTest
  find_package(GTest QUIET)

  if(NOT GTest_FOUND)
    # Download and build GTest if not found
    include(FetchContent)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.14.0
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  enable_testing()

  # NoC Overlay tests
  add_executable(noc_overlay_test
    tests/noc_overlay_test.cpp
    ${NOC_SIM_SOURCES}
  )

  target_include_directories(noc_overlay_test PRIVATE ${SYSTEMC_INCLUDE_DIR} src)
  # Use gtest (not gtest_main) since we define our own main in sc_main
  target_link_libraries(noc_overlay_test PRIVATE ${SYSTEMC_LIBRARY} GTest::gtest)

  include(GoogleTest)
  gtest_discover_tests(noc_overlay_test)
endif()

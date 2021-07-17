include(FetchContent)
FetchContent_Declare(gtest
  QUIET
  URL https://github.com/google/googletest/archive/release-1.10.0.tar.gz
)
# configure build of googletest
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(gtest)

# macto for testing
macro(add_unit_test MODULENAME TESTNAME)
  add_executable(
    ${TESTNAME} 
    ${TESTNAME}.cpp
  )
  target_link_libraries(
    ${TESTNAME}
    PRIVATE
    ${MODULENAME}
    gtest_main
  )
  include(GoogleTest)
gtest_discover_tests(
  ${TESTNAME}
  PROPERTIES
    LABELS "unit"
  DISCOVERY_TIMEOUT  # how long to wait (in seconds) before crashing
    240
)
endmacro()


# add tests folder
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/frenet_serret_converter)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/course_manager)




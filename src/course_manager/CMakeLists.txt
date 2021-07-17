
add_library(course_manager)

target_sources(course_manager
  PRIVATE
  course_manager.cpp
  PUBLIC
  course_manager.hpp
  )

target_include_directories(course_manager
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
  )

# we use this to get code coverage
# flags are only valid with the GNU compiler and on Linux
if(CMAKE_CXX_COMPILER_ID MATCHES GNU AND CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  target_compile_options(course_manager
    PUBLIC
      "--coverage"
    )
  target_link_options(course_manager
    INTERFACE
      "--coverage"
    )
endif()

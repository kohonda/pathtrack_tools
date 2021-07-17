
add_library(frenet_state_filter)

target_sources(frenet_state_filter
  PRIVATE
  frenet_state_filter.cpp
  PUBLIC
  frenet_state_filter.hpp
  )

target_include_directories(frenet_state_filter
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
  )

# we use this to get code coverage
# flags are only valid with the GNU compiler and on Linux
if(CMAKE_CXX_COMPILER_ID MATCHES GNU AND CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  target_compile_options(frenet_state_filter
    PUBLIC
      "--coverage"
    )
  target_link_options(frenet_state_filter
    INTERFACE
      "--coverage"
    )
endif()

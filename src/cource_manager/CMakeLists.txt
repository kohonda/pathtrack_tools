
add_library(cource_manager)

target_sources(cource_manager
  PRIVATE
  cource_manager.cpp
  PUBLIC
  cource_manager.hpp
  )

target_include_directories(cource_manager
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
  )

# we use this to get code coverage
# flags are only valid with the GNU compiler and on Linux
if(CMAKE_CXX_COMPILER_ID MATCHES GNU AND CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  target_compile_options(cource_manager
    PUBLIC
      "--coverage"
    )
  target_link_options(cource_manager
    INTERFACE
      "--coverage"
    )
endif()

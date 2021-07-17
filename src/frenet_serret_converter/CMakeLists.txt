
add_library(frenet_serret_converter)

target_sources(frenet_serret_converter
  PRIVATE
  frenet_serret_converter.cpp
  PUBLIC
  frenet_serret_converter.hpp
  )

target_include_directories(frenet_serret_converter
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
  )


# we use this to get code coverage
# flags are only valid with the GNU compiler and on Linux
if(CMAKE_CXX_COMPILER_ID MATCHES GNU AND CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  target_compile_options(frenet_serret_converter
    PUBLIC
      "--coverage"
    )
  target_link_options(frenet_serret_converter
    INTERFACE
      "--coverage"
    )
endif()

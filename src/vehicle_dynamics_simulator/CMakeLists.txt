
add_library(vehicle_dynamics_simulator)

target_sources(vehicle_dynamics_simulator
  PRIVATE
  vehicle_dynamics_simulator.cpp
  PUBLIC
  vehicle_dynamics_simulator.hpp
  )

target_include_directories(vehicle_dynamics_simulator
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
  )


# we use this to get code coverage
# flags are only valid with the GNU compiler and on Linux
if(CMAKE_CXX_COMPILER_ID MATCHES GNU AND CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  target_compile_options(vehicle_dynamics_simulator
    PUBLIC
      "--coverage"
    )
  target_link_options(vehicle_dynamics_simulator
    INTERFACE
      "--coverage"
    )
endif()

add_executable(example main.cpp)

target_include_directories(
  example
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
  )

target_link_libraries(
    example
    cource_manager
    frenet_serret_converter
    frenet_state_filter
    vehicle_dynamics_simulator
)

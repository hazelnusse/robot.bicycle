include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(reduced_order_observer control_design_functions.cc
                                      firmware_template.cc
                                      firmware_generator.cc
                                      reduced_order_observer.cc
                                      robot_bicycle_parameters.cc)

target_link_libraries(reduced_order_observer bicycle control)


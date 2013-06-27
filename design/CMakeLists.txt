include_directories(${EIGEN3_INCLUDE_DIR})
if (MATLAB_FOUND)
include_directories(${MATLAB_INCLUDE_DIR})
add_executable(full_order_observer control_design_functions.cc
                                      firmware_template.cc
                                      firmware_generator.cc
                                      matlab_interface.cc
                                      full_order_observer.cc
                                      robot_bicycle_parameters.cc)
target_link_libraries(full_order_observer bicycle control ${MATLAB_LIBRARIES})
endif()

add_executable(parameter_check parameter_check.cc robot_bicycle_parameters.cc)
target_link_libraries(parameter_check bicycle)

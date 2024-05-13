execute_process(COMMAND "/home/sa/RE510_2024/Experiment6/catkin_ws/build/simple_vehicle_sim/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sa/RE510_2024/Experiment6/catkin_ws/build/simple_vehicle_sim/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

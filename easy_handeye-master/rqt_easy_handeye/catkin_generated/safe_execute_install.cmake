execute_process(COMMAND "/home/robot/catkin_ws/src/easy_handeye-master/rqt_easy_handeye/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/robot/catkin_ws/src/easy_handeye-master/rqt_easy_handeye/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

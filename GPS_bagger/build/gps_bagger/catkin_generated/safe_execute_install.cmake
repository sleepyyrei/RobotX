execute_process(COMMAND "/home/song/Rei_WS/RobotX/GPS_bagger/build/gps_bagger/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/song/Rei_WS/RobotX/GPS_bagger/build/gps_bagger/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

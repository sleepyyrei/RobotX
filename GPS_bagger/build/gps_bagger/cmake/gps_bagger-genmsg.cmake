# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gps_bagger: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Imavros_msgs:/opt/ros/noetic/share/mavros_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Igeographic_msgs:/opt/ros/noetic/share/geographic_msgs/cmake/../msg;-Iuuid_msgs:/opt/ros/noetic/share/uuid_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gps_bagger_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv" NAME_WE)
add_custom_target(_gps_bagger_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gps_bagger" "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv" ""
)

get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv" NAME_WE)
add_custom_target(_gps_bagger_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gps_bagger" "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv" "sensor_msgs/NavSatFix:sensor_msgs/NavSatStatus:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_bagger
)
_generate_srv_cpp(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_bagger
)

### Generating Module File
_generate_module_cpp(gps_bagger
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_bagger
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gps_bagger_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gps_bagger_generate_messages gps_bagger_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_cpp _gps_bagger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_cpp _gps_bagger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_bagger_gencpp)
add_dependencies(gps_bagger_gencpp gps_bagger_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_bagger_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_bagger
)
_generate_srv_eus(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_bagger
)

### Generating Module File
_generate_module_eus(gps_bagger
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_bagger
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gps_bagger_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gps_bagger_generate_messages gps_bagger_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_eus _gps_bagger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_eus _gps_bagger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_bagger_geneus)
add_dependencies(gps_bagger_geneus gps_bagger_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_bagger_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_bagger
)
_generate_srv_lisp(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_bagger
)

### Generating Module File
_generate_module_lisp(gps_bagger
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_bagger
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gps_bagger_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gps_bagger_generate_messages gps_bagger_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_lisp _gps_bagger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_lisp _gps_bagger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_bagger_genlisp)
add_dependencies(gps_bagger_genlisp gps_bagger_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_bagger_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_bagger
)
_generate_srv_nodejs(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_bagger
)

### Generating Module File
_generate_module_nodejs(gps_bagger
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_bagger
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gps_bagger_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gps_bagger_generate_messages gps_bagger_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_nodejs _gps_bagger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_nodejs _gps_bagger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_bagger_gennodejs)
add_dependencies(gps_bagger_gennodejs gps_bagger_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_bagger_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_bagger
)
_generate_srv_py(gps_bagger
  "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_bagger
)

### Generating Module File
_generate_module_py(gps_bagger
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_bagger
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gps_bagger_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gps_bagger_generate_messages gps_bagger_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/callResponse.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_py _gps_bagger_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/song/Rei_WS/RobotX/GPS_bagger/src/gps_bagger/srv/WaypointService.srv" NAME_WE)
add_dependencies(gps_bagger_generate_messages_py _gps_bagger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gps_bagger_genpy)
add_dependencies(gps_bagger_genpy gps_bagger_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gps_bagger_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_bagger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gps_bagger
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gps_bagger_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(gps_bagger_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET mavros_msgs_generate_messages_cpp)
  add_dependencies(gps_bagger_generate_messages_cpp mavros_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_bagger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gps_bagger
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gps_bagger_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(gps_bagger_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET mavros_msgs_generate_messages_eus)
  add_dependencies(gps_bagger_generate_messages_eus mavros_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_bagger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gps_bagger
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gps_bagger_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(gps_bagger_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET mavros_msgs_generate_messages_lisp)
  add_dependencies(gps_bagger_generate_messages_lisp mavros_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_bagger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gps_bagger
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gps_bagger_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(gps_bagger_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET mavros_msgs_generate_messages_nodejs)
  add_dependencies(gps_bagger_generate_messages_nodejs mavros_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_bagger)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_bagger\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_bagger
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_bagger")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gps_bagger
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gps_bagger_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(gps_bagger_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET mavros_msgs_generate_messages_py)
  add_dependencies(gps_bagger_generate_messages_py mavros_msgs_generate_messages_py)
endif()

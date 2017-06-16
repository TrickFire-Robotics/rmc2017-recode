# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "daybreak_2k17: 2 messages, 0 services")

set(MSG_I_FLAGS "-Idaybreak_2k17:/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(daybreak_2k17_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg" NAME_WE)
add_custom_target(_daybreak_2k17_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "daybreak_2k17" "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg" NAME_WE)
add_custom_target(_daybreak_2k17_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "daybreak_2k17" "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/daybreak_2k17
)
_generate_msg_cpp(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/daybreak_2k17
)

### Generating Services

### Generating Module File
_generate_module_cpp(daybreak_2k17
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/daybreak_2k17
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(daybreak_2k17_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(daybreak_2k17_generate_messages daybreak_2k17_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_cpp _daybreak_2k17_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_cpp _daybreak_2k17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(daybreak_2k17_gencpp)
add_dependencies(daybreak_2k17_gencpp daybreak_2k17_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS daybreak_2k17_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/daybreak_2k17
)
_generate_msg_eus(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/daybreak_2k17
)

### Generating Services

### Generating Module File
_generate_module_eus(daybreak_2k17
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/daybreak_2k17
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(daybreak_2k17_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(daybreak_2k17_generate_messages daybreak_2k17_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_eus _daybreak_2k17_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_eus _daybreak_2k17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(daybreak_2k17_geneus)
add_dependencies(daybreak_2k17_geneus daybreak_2k17_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS daybreak_2k17_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/daybreak_2k17
)
_generate_msg_lisp(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/daybreak_2k17
)

### Generating Services

### Generating Module File
_generate_module_lisp(daybreak_2k17
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/daybreak_2k17
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(daybreak_2k17_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(daybreak_2k17_generate_messages daybreak_2k17_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_lisp _daybreak_2k17_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_lisp _daybreak_2k17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(daybreak_2k17_genlisp)
add_dependencies(daybreak_2k17_genlisp daybreak_2k17_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS daybreak_2k17_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/daybreak_2k17
)
_generate_msg_nodejs(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/daybreak_2k17
)

### Generating Services

### Generating Module File
_generate_module_nodejs(daybreak_2k17
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/daybreak_2k17
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(daybreak_2k17_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(daybreak_2k17_generate_messages daybreak_2k17_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_nodejs _daybreak_2k17_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_nodejs _daybreak_2k17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(daybreak_2k17_gennodejs)
add_dependencies(daybreak_2k17_gennodejs daybreak_2k17_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS daybreak_2k17_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/daybreak_2k17
)
_generate_msg_py(daybreak_2k17
  "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/daybreak_2k17
)

### Generating Services

### Generating Module File
_generate_module_py(daybreak_2k17
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/daybreak_2k17
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(daybreak_2k17_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(daybreak_2k17_generate_messages daybreak_2k17_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/DrivePacket.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_py _daybreak_2k17_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adamukaapan/catkin_ws/src/daybreak_2k17/msg/MotorOutputMsg.msg" NAME_WE)
add_dependencies(daybreak_2k17_generate_messages_py _daybreak_2k17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(daybreak_2k17_genpy)
add_dependencies(daybreak_2k17_genpy daybreak_2k17_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS daybreak_2k17_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/daybreak_2k17)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/daybreak_2k17
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(daybreak_2k17_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/daybreak_2k17)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/daybreak_2k17
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(daybreak_2k17_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/daybreak_2k17)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/daybreak_2k17
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(daybreak_2k17_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/daybreak_2k17)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/daybreak_2k17
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(daybreak_2k17_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/daybreak_2k17)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/daybreak_2k17\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/daybreak_2k17
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(daybreak_2k17_generate_messages_py std_msgs_generate_messages_py)
endif()

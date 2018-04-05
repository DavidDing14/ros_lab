# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "kinect2_viewer: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(kinect2_viewer_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/oslab/catkin_ws/src/iai_kinect2/kinect2_viewer/srv/UseStamp.srv" NAME_WE)
add_custom_target(_kinect2_viewer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kinect2_viewer" "/home/oslab/catkin_ws/src/iai_kinect2/kinect2_viewer/srv/UseStamp.srv" "sensor_msgs/PointField:sensor_msgs/PointCloud2:std_msgs/Header:sensor_msgs/Image"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(kinect2_viewer
  "/home/oslab/catkin_ws/src/iai_kinect2/kinect2_viewer/srv/UseStamp.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect2_viewer
)

### Generating Module File
_generate_module_cpp(kinect2_viewer
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect2_viewer
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(kinect2_viewer_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(kinect2_viewer_generate_messages kinect2_viewer_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oslab/catkin_ws/src/iai_kinect2/kinect2_viewer/srv/UseStamp.srv" NAME_WE)
add_dependencies(kinect2_viewer_generate_messages_cpp _kinect2_viewer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kinect2_viewer_gencpp)
add_dependencies(kinect2_viewer_gencpp kinect2_viewer_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kinect2_viewer_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(kinect2_viewer
  "/home/oslab/catkin_ws/src/iai_kinect2/kinect2_viewer/srv/UseStamp.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect2_viewer
)

### Generating Module File
_generate_module_lisp(kinect2_viewer
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect2_viewer
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(kinect2_viewer_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(kinect2_viewer_generate_messages kinect2_viewer_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oslab/catkin_ws/src/iai_kinect2/kinect2_viewer/srv/UseStamp.srv" NAME_WE)
add_dependencies(kinect2_viewer_generate_messages_lisp _kinect2_viewer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kinect2_viewer_genlisp)
add_dependencies(kinect2_viewer_genlisp kinect2_viewer_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kinect2_viewer_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(kinect2_viewer
  "/home/oslab/catkin_ws/src/iai_kinect2/kinect2_viewer/srv/UseStamp.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect2_viewer
)

### Generating Module File
_generate_module_py(kinect2_viewer
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect2_viewer
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(kinect2_viewer_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(kinect2_viewer_generate_messages kinect2_viewer_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oslab/catkin_ws/src/iai_kinect2/kinect2_viewer/srv/UseStamp.srv" NAME_WE)
add_dependencies(kinect2_viewer_generate_messages_py _kinect2_viewer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kinect2_viewer_genpy)
add_dependencies(kinect2_viewer_genpy kinect2_viewer_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kinect2_viewer_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect2_viewer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kinect2_viewer
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(kinect2_viewer_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(kinect2_viewer_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect2_viewer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kinect2_viewer
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(kinect2_viewer_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(kinect2_viewer_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect2_viewer)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect2_viewer\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kinect2_viewer
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(kinect2_viewer_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(kinect2_viewer_generate_messages_py sensor_msgs_generate_messages_py)
endif()

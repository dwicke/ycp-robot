FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/robot_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/SensorData.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_SensorData.lisp"
  "../msg_gen/lisp/MotorData.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MotorData.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

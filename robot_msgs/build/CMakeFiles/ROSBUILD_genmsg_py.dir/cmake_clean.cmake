FILE(REMOVE_RECURSE
  "../src/robot_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/robot_msgs/msg/__init__.py"
  "../src/robot_msgs/msg/_MotorData.py"
  "../src/robot_msgs/msg/_SensorData.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

FILE(REMOVE_RECURSE
  "../src/MotorControlMsg/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/MotorControlMsg/msg/__init__.py"
  "../src/MotorControlMsg/msg/_MotorCommand.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

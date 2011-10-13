cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(hogman_csparse
	src/csparse/cs_add.c
	src/csparse/cs_amd.c
	src/csparse/cs_chol.c
	src/csparse/cs_cholsol.c
	src/csparse/cs_compress.c
	src/csparse/cs_counts.c
	src/csparse/cs_cumsum.c
	src/csparse/cs_dfs.c
	src/csparse/cs_dmperm.c
	src/csparse/cs_droptol.c
	src/csparse/cs_dropzeros.c
	src/csparse/cs_dupl.c
	src/csparse/cs_entry.c
	src/csparse/cs_ereach.c
	src/csparse/cs_etree.c
	src/csparse/cs_fkeep.c
	src/csparse/cs_gaxpy.c
	src/csparse/cs_happly.c
	src/csparse/cs_house.c
	src/csparse/cs_ipvec.c
	src/csparse/cs_leaf.c
	src/csparse/cs_load.c
	src/csparse/cs_lsolve.c
	src/csparse/cs_ltsolve.c
	src/csparse/cs_lu.c
	src/csparse/cs_lusol.c
	src/csparse/cs_malloc.c
	src/csparse/cs_maxtrans.c
	src/csparse/cs_multiply.c
	src/csparse/cs_norm.c
	src/csparse/cs_permute.c
	src/csparse/cs_pinv.c
	src/csparse/cs_post.c
	src/csparse/cs_print.c
	src/csparse/cs_pvec.c
	src/csparse/cs_qr.c
	src/csparse/cs_qrsol.c
	src/csparse/cs_randperm.c
	src/csparse/cs_reach.c
	src/csparse/cs_scatter.c
	src/csparse/cs_scc.c
	src/csparse/cs_schol.c
	src/csparse/cs_spsolve.c
	src/csparse/cs_sqr.c
	src/csparse/cs_symperm.c
	src/csparse/cs_tdfs.c
	src/csparse/cs_transpose.c
	src/csparse/cs_updown.c
	src/csparse/cs_usolve.c
	src/csparse/cs_util.c
	src/csparse/cs_utsolve.c
)

rosbuild_add_library(hogman_stuff
	src/stuff/filesys_tools.cpp
	src/stuff/string_tools.cpp
	src/stuff/runtime_error.cpp
	src/stuff/os_specific.c
)

rosbuild_add_library(hogman_graph
	src/graph/graph.cpp
	src/graph/dijkstra.cpp
	src/graph/posegraph3d.cpp
	src/graph/loadEdges3d.cpp
	src/graph/posegraph2d.cpp
)

rosbuild_add_library(hogman_graph_optimizer
	src/graph_optimizer/graph_optimizer2d_aux.cpp
)

rosbuild_add_library(hogman_graph_optimizer_hogman
	src/graph_optimizer_hogman/csparse_helper.cpp
)

rosbuild_add_executable(hogman2d
	src/graph_optimizer_hogman/hogman2d.cpp
)

rosbuild_add_executable(hogman3d
	src/graph_optimizer_hogman/hogman3d.cpp
)
target_link_libraries(hogman3d hogman_csparse hogman_stuff hogman_graph hogman_graph_optimizer hogman_graph_optimizer_hogman)
target_link_libraries(hogman2d hogman_csparse hogman_stuff hogman_graph hogman_graph_optimizer hogman_graph_optimizer_hogman)

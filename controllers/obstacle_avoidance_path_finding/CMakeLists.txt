find_package(PythonLibs REQUIRED)

set(PYTHON_EXECUTABLE "/usr/bin/python2.7")
set(PYTHON_INCLUDE_DIR "/usr/include/python2.7")
set(PYTHON_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython2.7.so")
   
include_directories(${PYTHON_INCLUDE_DIRS})

add_library(obstacle_avoidance_path_finding MODULE obstacle_avoidance_path_finding.h ObjectTrackingNeuralNetwork.h obstacle_avoidance_path_finding.cpp ObjectTrackingNeuralNetwork.cpp)
target_link_libraries(obstacle_avoidance_path_finding 
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  ${PYTHON_LIBRARIES})

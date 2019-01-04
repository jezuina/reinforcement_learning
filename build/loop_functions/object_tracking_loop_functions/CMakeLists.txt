add_library(object_tracking_loop_functions MODULE 
  object_tracking_loop_functions.h
  object_tracking_loop_functions.cpp)
  
target_link_libraries(object_tracking_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot)



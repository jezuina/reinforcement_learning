# Install script for directory: /home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/footbot_diffusion/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/footbot_synchronization/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/footbot_flocking/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/footbot_gripping/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/footbot_foraging/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/footbot_manualcontrol/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/footbot_nn/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/eyebot_circle/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/eyebot_flocking/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/epuck_obstacleavoidance/cmake_install.cmake")
  include("/home/jkoroveshi/eclipse-workspace/argos3_examples@argos3-examples-master/controllers/obstacle_avoidance_path_finding/cmake_install.cmake")

endif()


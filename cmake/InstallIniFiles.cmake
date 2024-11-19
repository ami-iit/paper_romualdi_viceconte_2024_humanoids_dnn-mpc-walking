# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

macro(install_ini_files parent_dir)
  message(STATUS "Installing INI files from ${parent_dir}")
  file(GLOB network ${parent_dir}/*.ini)
  yarp_install(FILES ${network} DESTINATION ${DNN_MPC_DATA_INSTALL_DIR})
endmacro()

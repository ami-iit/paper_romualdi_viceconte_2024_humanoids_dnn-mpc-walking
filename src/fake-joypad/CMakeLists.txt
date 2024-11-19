# This software may be modified and distributed under the terms of the
# BSD 3-Clause License
# Authors: Giulio Romualdi

add_centroidal_mpc_walking_application(
  NAME            fake-joypad
  SOURCES         src/FakeJoypad.cpp src/main.cpp src/JoypadProvider.cpp
  HEADERS         include/FakeJoypad/FakeJoypad.h include/FakeJoypad/JoypadProvider.h
  LINK_LIBRARIES  ${SDL_LIBRARY} YARP::YARP_os YARP::YARP_init YARP::YARP_sig
                  BipedalLocomotion::System
                  BipedalLocomotion::ParametersHandlerYarpImplementation
                  BipedalLocomotion::SystemYarpImplementation
                  )

install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)

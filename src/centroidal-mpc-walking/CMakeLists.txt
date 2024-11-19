# This software may be modified and distributed under the terms of the
# BSD 3-Clause License
# Authors: Giulio Romualdi

add_centroidal_mpc_walking_application(
  NAME            walking
  SOURCES         src/main.cpp src/CentroidalMPCBlock.cpp src/WholeBodyQPBlock.cpp src/BaseEstimatorFromFootIMU.cpp src/CentroidalMPC.cpp
  HEADERS         include/CentroidalMPCWalking/CentroidalMPCBlock.h include/CentroidalMPCWalking/WholeBodyQPBlock.h include/CentroidalMPCWalking/BaseEstimatorFromFootIMU.h include/CentroidalMPCWalking/CentroidalMPC.h
  LINK_LIBRARIES  BipedalLocomotion::ParametersHandlerYarpImplementation
                  BipedalLocomotion::RobotInterfaceYarpImplementation
                  BipedalLocomotion::FloatingBaseEstimators
                  BipedalLocomotion::IK
                  BipedalLocomotion::ContinuousDynamicalSystem
                  BipedalLocomotion::System
                  BipedalLocomotion::SystemYarpImplementation
                  BipedalLocomotion::Planners
                  BipedalLocomotion::ManifConversions
                  BipedalLocomotion::ContactDetectors
                  BipedalLocomotion::SimplifiedModelControllers
                  BipedalLocomotion::ML
                  BipedalLocomotion::Math
                  BipedalLocomotion::GenericContainer
                  BipedalLocomotion::VectorsCollection
                  casadi
  )

install_robot_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)


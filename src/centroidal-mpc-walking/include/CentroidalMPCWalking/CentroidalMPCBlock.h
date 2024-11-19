/**
 * @file CentoidalMPCBlock.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#ifndef CENTROIDAL_MCP_WALKING_CENTROIDAL_MCP_BLOCK_H
#define CENTROIDAL_MCP_WALKING_CENTROIDAL_MCP_BLOCK_H

#include <map>
#include <memory>
#include <string>
#include <future>

#include <Eigen/Dense>

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ML/MANNAutoregressiveInputBuilder.h>
#include <BipedalLocomotion/ML/MANNTrajectoryGenerator.h>
#include <BipedalLocomotion/Math/LinearSpline.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <CentroidalMPCWalking/CentroidalMPC.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

namespace CentroidalMPCWalking
{

struct CentroidalMPCInput
{
    Eigen::Vector3d com;
    Eigen::Vector3d dcom;
    Eigen::Vector3d angularMomentum;
    BipedalLocomotion::Math::Wrenchd totalExternalWrench;
    std::chrono::nanoseconds currentTime;

    bool isValid{false};
};

struct CentroidalMPCOutputBlock
{
    CentroidalMPCWalking::CentroidalMPCOutput controllerOutput;
    BipedalLocomotion::Contacts::ContactPhaseList contactPhaseList;
    BipedalLocomotion::Contacts::ContactPhaseList mannContactPhaseList;
    std::chrono::nanoseconds currentTime{std::chrono::nanoseconds::zero()};
    std::chrono::nanoseconds mpcComputationTime{std::chrono::nanoseconds::zero()};
    std::chrono::nanoseconds adherentComputationTime{std::chrono::nanoseconds::zero()};
    Eigen::Vector3d comMANN;
    Eigen::Vector3d angularMomentumMann;
    Eigen::VectorXd regularizedJoints;
    Eigen::Vector2d facingDirection;
    Eigen::Vector2d motionDirection;

    bool isValid{false};
};

} // namespace CentroidalMPCWalking

namespace CentroidalMPCWalking
{
class CentroidalMPCBlock
    : public BipedalLocomotion::System::Advanceable<CentroidalMPCInput, CentroidalMPCOutputBlock>
{
    typename CentroidalMPCBlock::Output m_output;
    typename CentroidalMPCBlock::Input m_input;

    std::chrono::nanoseconds m_dT;
    std::chrono::nanoseconds m_absoluteTime{std::chrono::nanoseconds::zero()};

    CentroidalMPCWalking::CentroidalMPC m_controller;
    BipedalLocomotion::ML::MANNAutoregressiveInputBuilder m_generatorInputBuilder;
    BipedalLocomotion::ML::MANNTrajectoryGenerator m_generator;
    BipedalLocomotion::ML::MANNDirectionalInput m_directionalInput;

    struct FrequencyAdapter
    {
        BipedalLocomotion::Math::LinearSpline<Eigen::Vector3d> spline;
        std::vector<std::chrono::nanoseconds> inputTimeKnots;
        std::vector<std::chrono::nanoseconds> outputTimeKnots;
        std::vector<std::chrono::nanoseconds> inputTimeKnotsAbsolute;
        std::vector<std::chrono::nanoseconds> outputTimeKnotsAbsolute;
        std::vector<Eigen::Vector3d> outputPoints;
        std::vector<Eigen::Vector3d> dummy;
    };

    FrequencyAdapter m_comFrequencyAdapter;
    FrequencyAdapter m_angularMomentumFrequencyAdapter;

    std::size_t m_mergePointIndex{1};
    std::chrono::nanoseconds m_mannCallingTime{std::chrono::nanoseconds::zero()};

    yarp::os::BufferedPort<yarp::sig::Vector> m_joypadPort;

    BipedalLocomotion::Contacts::ContactPhaseList m_phaseList;

    bool m_inputValid{false};
    double m_currentTime{0};
    BipedalLocomotion::Contacts::ContactPhaseList::const_iterator m_phaseIt;
    bool m_isFirstRun{true};
    Eigen::MatrixXd m_comTraj;
    unsigned int m_indexCoM{0};
    double m_robotMass;

    std::future<bool> m_mannFuture;
    bool runMANNAsynchronously();
    BipedalLocomotion::ML::MANNTrajectoryGenerator::Output m_MANNGeneratorOutput;

public:
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                        handler) override;

    const Output& getOutput() const override;

    bool setInput(const Input& input) override;

    bool advance() override;

    bool isOutputValid() const override;
};
} // namespace CentroidalMPCWalking

#endif // CENTROIDAL_MCP_WALKING_CENTROIDAL_MCP_BLOCK_H

/**
 * @file WholeBodyQPBlock.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#ifndef CENTROIDAL_MCP_WALKING_WHOLE_BODY_QP_BLOCK_H
#define CENTROIDAL_MCP_WALKING_WHOLE_BODY_QP_BLOCK_H

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>

#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/CentroidalDynamics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/RK4.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/MultiStateWeightProvider.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ButterworthLowPassFilter.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>

#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointLimitsTask.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/R3Task.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/IK/AngularMomentumTask.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/SimplifiedModelControllers/CoMZMPController.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

#include <CentroidalMPCWalking/CentroidalMPCBlock.h>
#include <CentroidalMPCWalking/BaseEstimatorFromFootIMU.h>


namespace CentroidalMPCWalking
{

template <typename T> class FirstOrderLowPassFilter
{
    T m_previousValue;
    bool m_firstIteration{true};
    double m_alpha;

public:
    FirstOrderLowPassFilter(double cutoffFreq, double samplingTime)
    {
        double tau = 1.0 / (2.0 * M_PI * cutoffFreq);
        m_alpha = samplingTime / (tau + samplingTime);
        m_firstIteration = true;
    }

    T filter(const T& value)
    {
        if (m_firstIteration)
        {
            m_previousValue = value;
            m_firstIteration = false;
            return m_previousValue;
        }

        m_previousValue = m_alpha * value + (1.0 - m_alpha) * m_previousValue;

        return m_previousValue;
    }

    void reset()
    {
        m_firstIteration = true;
    }
};

class WholeBodyQPBlock
    : public BipedalLocomotion::System::Advanceable<CentroidalMPCOutputBlock, CentroidalMPCInput>
{

    typename WholeBodyQPBlock::Output m_output;
    typename WholeBodyQPBlock::Input m_input;
    bool m_firstIteration{true};
    bool m_firstTorqueSet{true};
    double m_robotMass;
    /*     std::string m_robot; /**< Robot name. */

    Eigen::VectorXd m_currentJointPos; /**< Current joint positions. */
    Eigen::VectorXd m_currentJointVel; /**< Current joint velocities. */
    // std::shared_ptr<FirstOrderLowPassFilter<Eigen::VectorXd>> m_jointVelFilter;
    BipedalLocomotion::ContinuousDynamicalSystem::ButterworthLowPassFilter m_jointVelFilter;
    Eigen::VectorXd m_desJointPos; /**< Current joint positions. */
    Eigen::VectorXd m_desJointVel; /**< Current joint velocities. */

    Eigen::VectorXd m_desJointPosForRobot;
    Eigen::VectorXd m_currentJointPosWithoutNeck;
    Eigen::VectorXd m_currentJointVelWithoutNeck;

    std::vector<std::string> m_jointsList;

    bool m_filterJointVel{false};

    BipedalLocomotion::YarpUtilities::VectorsCollectionServer m_logDataServer;

    manif::SE3d m_baseTransform;
    manif::SE3d::Tangent m_baseVelocity;
    manif::SE3d m_baseTransformWithIMU;
    manif::SE3d::Tangent m_baseVelocityWithIMU;

    BipedalLocomotion::RobotInterface::PolyDriverDescriptor m_controlBoard; /**< Control board
                                                                               remapper. */

    struct ContactWrenchHandler
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor polyDriverDescriptor;
        BipedalLocomotion::Math::Wrenchd wrench;
    };

    struct JointRegularizationAnkleStrategy
    {
        int jointIndex;
        double desired;
        double admittanceGain{2.5};
    };

    JointRegularizationAnkleStrategy m_leftAnkleRoll;
    JointRegularizationAnkleStrategy m_leftAnklePitch;
    JointRegularizationAnkleStrategy m_rightAnkleRoll;
    JointRegularizationAnkleStrategy m_rightAnklePitch;

    std::unordered_map<std::string, ContactWrenchHandler> m_leftFootContacWrenches;
    std::unordered_map<std::string, ContactWrenchHandler> m_rightFootContacWrenches;
    std::unordered_map<std::string, ContactWrenchHandler> m_externalContactWrenches;

    BipedalLocomotion::RobotInterface::YarpRobotControl m_robotControl; /**< Robot control object.
                                                                         */
    BipedalLocomotion::RobotInterface::YarpSensorBridge m_sensorBridge; /**< Sensor bridge object.
                                                                         */

    BipedalLocomotion::Estimators::LeggedOdometry m_floatingBaseEstimator;
    BipedalLocomotion::Contacts::FixedFootDetector m_fixedFootDetector;

    BipedalLocomotion::Planners::SwingFootPlanner m_leftFootPlanner;
    BipedalLocomotion::Planners::SwingFootPlanner m_rightFootPlanner;

    BipedalLocomotion::SimplifiedModelControllers::CoMZMPController m_CoMZMPController;
    struct IKProblemAndTask
    {
        BipedalLocomotion::IK::IntegrationBasedIKProblem ikProblem;
        std::shared_ptr<BipedalLocomotion::IK::SE3Task> leftFootTask;
        std::shared_ptr<BipedalLocomotion::IK::SE3Task> rightFootTask;
        std::shared_ptr<BipedalLocomotion::IK::CoMTask> comTask;
        std::shared_ptr<BipedalLocomotion::IK::SO3Task> chestTask;
        std::shared_ptr<BipedalLocomotion::IK::R3Task> rootTask;
        std::shared_ptr<BipedalLocomotion::IK::JointTrackingTask> regularizationTask;
        std::shared_ptr<BipedalLocomotion::IK::JointLimitsTask> jointLimitsTask;
        std::shared_ptr<BipedalLocomotion::IK::SE3Task> baseTask;
        std::shared_ptr<BipedalLocomotion::IK::AngularMomentumTask> angularMomentumTask;

        // weights
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider> jointRegularizationWeight;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider> leftFootWeight;
        std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider> rightFootWeight;        
    };
    IKProblemAndTask m_IKandTasks;


    Eigen::Vector3d m_rootLinkOffset;
    Eigen::VectorXd m_jointPosRegularize;

    template <typename _Dynamics, typename _Integrator> struct DynamicsAndIntegrator
    {
        std::shared_ptr<_Integrator> integrator;
        std::shared_ptr<_Dynamics> dynamics;
    };

    DynamicsAndIntegrator<BipedalLocomotion::ContinuousDynamicalSystem::CentroidalDynamics,
                          BipedalLocomotion::ContinuousDynamicalSystem::RK4<
                              BipedalLocomotion::ContinuousDynamicalSystem::CentroidalDynamics>>
        m_centroidalSystem;

    DynamicsAndIntegrator<
        BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem,
        BipedalLocomotion::ContinuousDynamicalSystem::RK4<
            BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem>>
        m_comSystem;

    DynamicsAndIntegrator<
        BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem,
        BipedalLocomotion::ContinuousDynamicalSystem::RK4<
            BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem>>
        m_floatingBaseSystem;

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynWithDesired;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynWithMeasured;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynWithRegularization;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynJointsDesiredBaseMeasured;

    std::chrono::nanoseconds m_dT;
    std::chrono::nanoseconds m_absoluteTime{std::chrono::nanoseconds::zero()};

    CentroidalMPCWalking::BaseEstimatorFromFootIMU m_baseEstimatorFromFootIMU;

    struct IMUOrientationData
    {
        iDynTree::FrameIndex imuFrameIndex;
        iDynTree::Rotation sole_R_link;
        iDynTree::Rotation link_R_IMU;
        iDynTree::Rotation IMU_R_sole;
        iDynTree::Rotation I_R_I_IMU;
        iDynTree::Rotation I_R_IMU;
        iDynTree::Rotation I_R_sole;
        Eigen::Vector3d angularVelocity;
    /*         std::shared_ptr<FirstOrderLowPassFilter<Eigen::Vector3d>> angularVelocityFilter; */
        BipedalLocomotion::ContinuousDynamicalSystem::ButterworthLowPassFilter angularVelocityFilter;
        std::string gyroName;
        std::string orientationName;
    };

    struct linkIMU
    {
        std::unordered_map<std::string, IMUOrientationData> IMUs;
        iDynTree::Rotation averageRotation;
        Eigen::Vector3d averageAngularVelocity;
        Eigen::Vector3d originalAverageAngularVelocity;
        iDynTree::Rotation soleInit_R_I;
        iDynTree::FrameIndex soleFrameIndex;
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor polyDriverDescriptor;
    };

    std::unordered_map<std::string, linkIMU> m_linksWithIMU;
    BipedalLocomotion::RobotInterface::PolyDriverDescriptor m_masRemapper;

    std::unordered_map<std::string, std::pair<const std::string, const manif::SE3d>>
        m_baseFrames; /**< Transform related to the base frame */

    bool setBaseFrame(const std::string& baseFrame,
                      const std::string& name,
                      const iDynTree::Model& model);

    bool m_prevContactLeft{false};

    bool createPolydriver(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool initializeRobotControl(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool instantiateSensorBridge(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool instantiateIK(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool instantiateLeggedOdometry(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
        const std::string& modelPath,
        const std::vector<std::string>& jointLists);

    bool instantiateSwingFootPlanner(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool createKinDyn(const std::string& modelPath, const std::vector<std::string>& jointLists);

    bool updateFloatingBase();

    bool createAllContactWrenchesDriver(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    BipedalLocomotion::RobotInterface::PolyDriverDescriptor createContactWrenchDriver(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
        const std::string& local);

    bool evaluateZMP(Eigen::Ref<Eigen::Vector2d> zmp,
                     Eigen::Ref<Eigen::Vector2d> localLeft,
                     Eigen::Ref<Eigen::Vector2d> localRight,
                     bool& isLocalLeftDefined,
                     bool& isLocalRightDefine);

    bool computeDesiredZMP(
        const std::map<std::string, BipedalLocomotion::Contacts::DiscreteGeometryContact>& contacts,
        Eigen::Ref<Eigen::Vector2d> zmp,
        Eigen::Ref<Eigen::Vector2d> localLeft,
        Eigen::Ref<Eigen::Vector2d> localRight,
        bool& isLocalLeftDefined,
        bool& isLocalRightDefine);

    bool configureLinkWithIMU(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
        const std::string& linkName,
        const std::string& localPrefix);

    bool configureLinksWithIMU(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool m_mpcActsAsPlanner{false};
    bool m_useIMUBaseEstimatorForKindynMeasured{false};
    bool m_useIMUBaseEstimatorForKindynDesired{false};
    bool m_useIMUBaseVelocityForKindynDesired{false};
    bool m_useIMUBaseVelocityForKindynMeasured{false};
    bool m_filterGyroscope{false};
    bool m_useNoControlForContactFoot{false};
    bool m_enableAnkleStrategy{false};
    bool m_useDifferentGainsForStanceFoot{false};
    bool m_useLocalAdjustmentJointAnkles{false};
    bool m_enableCoMZMPController{false};
    bool m_disableBaseControlForSomeIKTasks{false};
    bool m_useMeasuredBaseVelocityForIK{false};

    std::vector<double> m_flags;

public:
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                        handler) override;

    const Output& getOutput() const override;

    bool setInput(const Input& input) override;

    bool advance() override;

    bool isOutputValid() const override;

    bool close() override;
};
} // namespace CentroidalMPCWalking

#endif // CENTROIDAL_MCP_WALKING_WHOLE_BODY_QP_BLOCK_H

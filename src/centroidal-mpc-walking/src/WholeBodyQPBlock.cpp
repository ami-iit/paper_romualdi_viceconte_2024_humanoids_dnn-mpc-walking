/**
 * @file WholeBodyQPBlock.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include <unordered_set>

#include <Eigen/Dense>

#include <manif/SE3.h>

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/SO3Utils.h>
#include <iDynTree/SpatialMomentum.h>

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/Contacts/ContactListJsonParser.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <CentroidalMPCWalking/WholeBodyQPBlock.h>

using namespace CentroidalMPCWalking;
using namespace BipedalLocomotion::ParametersHandler;
using namespace std::chrono_literals;

double extactYawAngle(const Eigen::Ref<const Eigen::Matrix3d>& R)
{
    if (R(2, 0) < 1.0)
    {
        if (R(2, 0) > -1.0)
        {
            return atan2(R(1, 0), R(0, 0));
        } else
        {
            // Not a unique solution
            return -atan2(-R(1, 2), R(1, 1));
        }
    }

    // Not a unique solution
    return atan2(-R(1, 2), R(1, 1));
}

Eigen::Vector2d saturateLocalZMP(Eigen::Ref<const Eigen::Vector2d> localZMP)
{
    constexpr double foot_width_in_m = 0.097071067812;
    constexpr double foot_length_in_m = 0.233321067812;

    Eigen::Vector2d saturatedLocalZMP = localZMP;
    saturatedLocalZMP(0)
        = std::min(foot_length_in_m / 2, std::max(-foot_length_in_m / 2, localZMP(0)));
    saturatedLocalZMP(1)
        = std::min(foot_width_in_m / 2, std::max(-foot_width_in_m / 2, localZMP(1)));
    return saturatedLocalZMP;
}

std::unordered_set<int> findCornerNotActive(Eigen::Ref<const Eigen::Vector2d> localZMP)
{
    constexpr double foot_width_in_m = 0.097071067812;
    constexpr double foot_length_in_m = 0.233321067812;

    constexpr double foot_width_half = foot_width_in_m / 2;
    constexpr double foot_length_half = foot_length_in_m / 2;

    constexpr double tolerance = 0.001;
    std::unordered_set<int> nonActiveContactCorners;

    // if the local ZMP is far from the edges of the foot, all the corners are active
    if (std::abs(localZMP(0)) < foot_length_half - tolerance
        && std::abs(localZMP(1)) < foot_width_half - tolerance)
    {
        return nonActiveContactCorners;
    } else
    {
        // if the local zmp is close to the edges of the foot, some corners are not active
        // we have several possible cases
        // case 1: the local ZMP is next to the upper edge of the foot in this case the corners 0
        // and 1 are active and the corners 2 and 3 are not active
        if (std::abs(localZMP(0) - foot_length_half) < tolerance)
        {
            nonActiveContactCorners.insert(2);
            nonActiveContactCorners.insert(3);
        }

        // case 2: the local ZMP is next to the lower edge of the foot in this case the corners 2
        // and 3 are active and the corners 0 and 1 are not active.

        else if (std::abs(localZMP(0) + foot_length_half) < tolerance)
        {
            nonActiveContactCorners.insert(0);
            nonActiveContactCorners.insert(1);
        }

        // case 3: the local ZMP is next to the left edge of the foot in this case the corners 0 and
        // 2 are active and the corners 1 and 3 are not active
        if (std::abs(localZMP(1) - foot_width_half) < tolerance)
        {
            nonActiveContactCorners.insert(1);
            nonActiveContactCorners.insert(2);
        }

        // case 4: the local ZMP is next to the right edge of the foot in this case the corners 1
        // and 3 are active and the corners 0 and 2 are not active
        else if (std::abs(localZMP(1) + foot_width_half) < tolerance)
        {
            nonActiveContactCorners.insert(0);
            nonActiveContactCorners.insert(3);
        }

        return nonActiveContactCorners;
    }
}

bool WholeBodyQPBlock::setBaseFrame(const std::string& baseFrame,
                                    const std::string& name,
                                    const iDynTree::Model& model)
{
    if (!model.isValid())
    {
        BipedalLocomotion::log()->error("[WholeBodyQPBlock::setBaseFrame] The model is not valid.");
        return false;
    }

    auto frameBaseIndex = model.getFrameIndex(baseFrame);
    if (frameBaseIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        BipedalLocomotion::log()->error("[WholeBodyQPBlock::setBaseFrame] Unable to find the frame "
                                        "named {}.",
                                        baseFrame);
        return false;
    }

    auto linkBaseIndex = model.getFrameLink(frameBaseIndex);
    if (linkBaseIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        BipedalLocomotion::log()->error("[WholeBodyQPBlock::setBaseFrame] Unable to find the link "
                                        "associated to the frame named {}.",
                                        baseFrame);
        return false;
    }

    std::string baseFrameName = model.getLinkName(linkBaseIndex);

    m_baseFrames.insert({name,
                         std::make_pair(baseFrameName,
                                        BipedalLocomotion::Conversions::toManifPose(
                                            model.getFrameTransform(frameBaseIndex).inverse()))});
    return true;
}

bool WholeBodyQPBlock::configureLinksWithIMU(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::configureLinksWithIMU]";

    auto tmp = handler.lock();
    if (tmp == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    std::string localPrefix;
    if (!tmp->getParameter("name", localPrefix))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", logPrefix);
        return false;
    }

    auto params = tmp->getGroup("LINKS_WITH_IMU").lock();
    if (params == nullptr)
    {
        BipedalLocomotion::log()->error("{} Unable to find the group 'LINKS_WITH_IMU'.", logPrefix);
        return false;
    }

    // print the params
    if (!this->configureLinkWithIMU(params->getGroup("LEFT_FOOT"), "left_foot", localPrefix))
    {
        BipedalLocomotion::log()->error("{} Unable to configure the left foot IMU.", logPrefix);
        return false;
    }

    if (!this->configureLinkWithIMU(params->getGroup("RIGHT_FOOT"), "right_foot", localPrefix))
    {
        BipedalLocomotion::log()->error("{} Unable to configure the right foot IMU.", logPrefix);
        return false;
    }

    std::vector<BipedalLocomotion::RobotInterface::PolyDriverDescriptor> polyDrivers;
    polyDrivers.push_back(m_linksWithIMU["left_foot"].polyDriverDescriptor);
    polyDrivers.push_back(m_linksWithIMU["right_foot"].polyDriverDescriptor);
    m_masRemapper = //
        BipedalLocomotion::RobotInterface::constructMultipleAnalogSensorsRemapper( //
            params->getGroup("MULTIPLE_ANALOG_SENSORS_REMAPPER"),
            polyDrivers);

    return m_masRemapper.isValid();
}

bool WholeBodyQPBlock::configureLinkWithIMU(std::weak_ptr<const IParametersHandler> handler,
                                            const std::string& linkName,
                                            const std::string& localPrefix)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::configureLinkWithIMU]";

    auto tmp = handler.lock();
    if (tmp == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    auto linkHandler = tmp->clone();
    linkHandler->setParameter("local_prefix", localPrefix);

    std::string tempDescription;
    if (!linkHandler->getParameter("description", tempDescription))
    {
        BipedalLocomotion::log()->error("{} Unable to get the description of the IMU.", logPrefix);
        return false;
    }

    auto linkPoly
        = BipedalLocomotion::RobotInterface::constructMultipleAnalogSensorsClient(linkHandler);
    if (!linkPoly.isValid())
    {
        BipedalLocomotion::log()->error("{} Unable to create the polydriver for the IMU.",
                                        logPrefix);
        return false;
    }

    std::vector<std::string> imuNames;
    if (!linkHandler->getParameter("imu_names", imuNames))
    {
        BipedalLocomotion::log()->error("{} Unable to get the IMU names associated to {} IMUs",
                                        logPrefix,
                                        tempDescription);
        return false;
    }

    std::vector<std::string> orientationNames;
    if (!linkHandler->getParameter("orientation_names", orientationNames))
    {
        BipedalLocomotion::log()->error("{} Unable to get the orientation names associated to {} "
                                        "IMUs",
                                        logPrefix,
                                        tempDescription);
        return false;
    }

    std::vector<std::string> gyroNames;
    if (!linkHandler->getParameter("gyro_names", gyroNames))
    {
        BipedalLocomotion::log()->error("{} Unable to get the gyro names associated to {} IMUs",
                                        logPrefix,
                                        tempDescription);
        return false;
    }

    std::vector<std::string> frameNames;
    if (!linkHandler->getParameter("frame_names", frameNames))
    {
        BipedalLocomotion::log()->error("{} Unable to get the frame names associated to {} IMUs",
                                        logPrefix,
                                        tempDescription);
        return false;
    }

    std::string soleFrameName;
    if (!linkHandler->getParameter("sole_frame_name", soleFrameName))
    {
        BipedalLocomotion::log()->error("{} Unable to get the sole frame name associated to the {} "
                                        "IMUs",
                                        logPrefix,
                                        tempDescription);
        return false;
    }

    iDynTree::FrameIndex soleFrameIndex
        = m_kinDynWithMeasured->model().getFrameIndex(soleFrameName);
    if (soleFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        BipedalLocomotion::log()->error("{} Unable to find the sole frame named {}",
                                        logPrefix,
                                        soleFrameName);
        return false;
    }
    m_linksWithIMU[linkName].soleFrameIndex = soleFrameIndex;

    const iDynTree::Rotation link_R_sole
        = m_kinDynWithMeasured->model().getFrameTransform(soleFrameIndex).getRotation();

    if (imuNames.size() != frameNames.size())
    {
        BipedalLocomotion::log()->error("{} The number of IMU names and frame names should be the "
                                        "same for the link named {}",
                                        logPrefix,
                                        linkName);
        return false;
    }

    if (imuNames.size() != orientationNames.size())
    {
        BipedalLocomotion::log()->error("{} The number of IMU names and orientation names should "
                                        "be "
                                        "the same for the link named {}",
                                        logPrefix,
                                        linkName);
        return false;
    }

    if (imuNames.size() != gyroNames.size())
    {
        BipedalLocomotion::log()->error("{} The number of IMU names and gyro names should be the "
                                        "same for the link named {}",
                                        logPrefix,
                                        linkName);
        return false;
    }

    for (int i = 0; i < imuNames.size(); i++)
    {
        m_linksWithIMU[linkName].IMUs[imuNames[i]].imuFrameIndex
            = m_kinDynWithMeasured->model().getFrameIndex(frameNames[i]);

        m_linksWithIMU[linkName].IMUs[imuNames[i]].orientationName = orientationNames[i];
        m_linksWithIMU[linkName].IMUs[imuNames[i]].gyroName = gyroNames[i];

        if (m_linksWithIMU[linkName].IMUs[imuNames[i]].imuFrameIndex
            == iDynTree::FRAME_INVALID_INDEX)
        {
            BipedalLocomotion::log()->error("{} Unable to find the frame named {}",
                                            logPrefix,
                                            frameNames[i]);
            return false;
        }

        // check that the IMU frame and the sole frame belong to the same link
        if (m_kinDynWithMeasured->model().getFrameLink(soleFrameIndex)
            != m_kinDynWithMeasured->model().getFrameLink(
                m_linksWithIMU[linkName].IMUs[imuNames[i]].imuFrameIndex))
        {
            BipedalLocomotion::log()->error("{} The IMU frame and the sole frame do not belong to "
                                            "the same link for the link named {}",
                                            logPrefix,
                                            linkName);
            return false;
        }

        // covnert m_dT in double seconds
        double dT = std::chrono::duration<double>(m_dT).count();

        if (m_filterGyroscope)
        {
            if (!m_linksWithIMU[linkName].IMUs[imuNames[i]].angularVelocityFilter.initialize(
                    linkHandler))
            {
                BipedalLocomotion::log()->error("{} Unable to initialize the angular velocity "
                                                "filter "
                                                "for the IMU named {}",
                                                logPrefix,
                                                imuNames[i]);
                return false;
            }

            m_linksWithIMU[linkName].IMUs[imuNames[i]].angularVelocityFilter.reset(
                Eigen::Vector3d::Zero());
        }

        const iDynTree::Rotation link_R_IMU
            = m_kinDynWithMeasured->model()
                  .getFrameTransform(m_linksWithIMU[linkName].IMUs[imuNames[i]].imuFrameIndex)
                  .getRotation();

        m_linksWithIMU[linkName].IMUs[imuNames[i]].IMU_R_sole = link_R_IMU.inverse() * link_R_sole;
    }

    m_linksWithIMU[linkName].polyDriverDescriptor = linkPoly;

    return true;
}

bool WholeBodyQPBlock::createKinDyn(const std::string& modelPath,
                                    const std::vector<std::string>& jointLists)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::createKinDyn]";

    iDynTree::ModelLoader ml;
    if (!ml.loadReducedModelFromFile(modelPath, jointLists))
    {
        BipedalLocomotion::log()->error("{} Unable to load the reduced model located in: {}.",
                                        errorPrefix,
                                        modelPath);
        return false;
    }

    m_kinDynWithDesired = std::make_shared<iDynTree::KinDynComputations>();
    m_kinDynWithMeasured = std::make_shared<iDynTree::KinDynComputations>();
    m_kinDynWithRegularization = std::make_shared<iDynTree::KinDynComputations>();
    m_kinDynJointsDesiredBaseMeasured = std::make_shared<iDynTree::KinDynComputations>();

    if (!m_kinDynWithDesired->loadRobotModel(ml.model())
        || !m_kinDynWithMeasured->loadRobotModel(ml.model())
        || !m_kinDynWithRegularization->loadRobotModel(ml.model())
        || !m_kinDynJointsDesiredBaseMeasured->loadRobotModel(ml.model()))
    {
        BipedalLocomotion::log()->error("{} Unable to load a KinDynComputation object",
                                        errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateLeggedOdometry(std::shared_ptr<const IParametersHandler> handler,
                                                 const std::string& modelPath,
                                                 const std::vector<std::string>& jointLists)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::instantiateLeggedOdometry]";

    iDynTree::ModelLoader ml;
    if (!ml.loadReducedModelFromFile(modelPath, jointLists))
    {
        BipedalLocomotion::log()->error("{} Unable to load the reduced model located in: {}.",
                                        logPrefix,
                                        modelPath);
        return false;
    }

    auto tmpKinDyn = std::make_shared<iDynTree::KinDynComputations>();
    if (!tmpKinDyn->loadRobotModel(ml.model()))
    {
        BipedalLocomotion::log()->error("{} Unable to load a KinDynComputation object", logPrefix);
        return false;
    }

    if (!m_floatingBaseEstimator.initialize(handler->getGroup("FLOATING_BASE_ESTIMATOR"),
                                            tmpKinDyn))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the legged odometry.", logPrefix);
        return false;
    }

    if (!m_fixedFootDetector.initialize(handler->getGroup("FIXED_FOOT_DETECTOR")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the fixed foot detector.",
                                        logPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateIK(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::instantiateIK]";

    auto getWeightProvider
        = [this, logPrefix](const std::string& taskName, auto& weightProvider) -> bool {
        auto it = m_IKandTasks.ikProblem.weights.find(taskName);
        if (it == m_IKandTasks.ikProblem.weights.end())
        {
            BipedalLocomotion::log()->error("{} Unable to get the weight provider named {}.",
                                            logPrefix,
                                            taskName);
            return false;
        }

        auto ptr = it->second;

        if (ptr == nullptr)
        {
            BipedalLocomotion::log()->error("{} Unable to get the weight provider named {}.",
                                            logPrefix,
                                            taskName);
            return false;
        }

        // cast the weight provider
        weightProvider = std::dynamic_pointer_cast<
            typename std::remove_reference<decltype(weightProvider)>::type::element_type>(ptr);

        if (weightProvider == nullptr)
        {
            BipedalLocomotion::log()->error("{} Unable to cast the weight provider named {} to "
                                            "the expected type.",
                                            logPrefix,
                                            taskName);
            return false;
        }
        return true;
    };

    auto getTask = [this, logPrefix](const std::string& taskName, auto& task) -> bool {
        auto ptr = m_IKandTasks.ikProblem.ik->getTask(taskName).lock();
        if (ptr == nullptr)
        {
            BipedalLocomotion::log()->error("{} Unable to get the task named {}.",
                                            logPrefix,
                                            taskName);
            return false;
        }

        // cast the task
        task = std::dynamic_pointer_cast<
            typename std::remove_reference<decltype(task)>::type::element_type>(ptr);
        if (task == nullptr)
        {
            BipedalLocomotion::log()->error("{} Unable to cast the task named {} to the expected "
                                            "type.",
                                            logPrefix,
                                            taskName);
            return false;
        }

        return true;
    };

    m_IKandTasks.ikProblem
        = BipedalLocomotion::IK::QPInverseKinematics::build(handler, m_kinDynWithDesired);
    if (!m_IKandTasks.ikProblem.isValid())
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the IK.", logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{}", m_IKandTasks.ikProblem.ik->toString());

    // attempt to get the joint limits task
    if (!getTask("JOINT_LIMITS_TASK", m_IKandTasks.jointLimitsTask))
    {
        BipedalLocomotion::log()->info("{} Unable to get the joint limits task.", logPrefix);
    }

    if (!getTask("ANGULAR_MOMENTUM_TASK", m_IKandTasks.angularMomentumTask))
    {
        BipedalLocomotion::log()->info("{} Unable to get the angular momentum task.", logPrefix);
    }

    // get the weight provider for the feet tasks and the joint regularization task
    if (!getWeightProvider("LEFT_FOOT", m_IKandTasks.leftFootWeight))
    {
        BipedalLocomotion::log()->warn("{} Unable to get the weight provider for the left foot.",
                                       logPrefix);
    }

    if (!getWeightProvider("RIGHT_FOOT", m_IKandTasks.rightFootWeight))
    {
        BipedalLocomotion::log()->warn("{} Unable to get the weight provider for the right foot.",
                                       logPrefix);
    }

    if (!getWeightProvider("JOINT_REGULARIZATION", m_IKandTasks.jointRegularizationWeight))
    {
        BipedalLocomotion::log()->warn("{} Unable to get the weight provider for the COM.",
                                       logPrefix);
    }

    return getTask("LEFT_FOOT", m_IKandTasks.leftFootTask)
           && getTask("RIGHT_FOOT", m_IKandTasks.rightFootTask)
           && getTask("COM", m_IKandTasks.comTask) //
           && getTask("CHEST", m_IKandTasks.chestTask)
           && getTask("JOINT_REGULARIZATION", m_IKandTasks.regularizationTask)
           && getTask("ROOT_TASK", m_IKandTasks.rootTask)
           && getTask("BASE_TASK", m_IKandTasks.baseTask);
}

bool WholeBodyQPBlock::initializeRobotControl(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::initializeRobotControl]";

    if (!m_robotControl.initialize(handler->getGroup("ROBOT_CONTROL")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the robot control.", errorPrefix);
        return false;
    }
    if (!m_robotControl.setDriver(m_controlBoard.poly))
    {
        BipedalLocomotion::log()->error("{} Unable to set the polydriver.", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateSensorBridge(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::instantiateSensorBridge]";

    if (!m_sensorBridge.initialize(handler->getGroup("SENSOR_BRIDGE")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the sensor bridge.", errorPrefix);
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(m_controlBoard.poly.get(), m_controlBoard.key.c_str());
    for (auto& [key, driver] : m_leftFootContacWrenches)
    {
        list.push(driver.polyDriverDescriptor.poly.get(), key.c_str());
    }

    for (auto& [key, driver] : m_rightFootContacWrenches)
    {
        list.push(driver.polyDriverDescriptor.poly.get(), key.c_str());
    }

    for (auto& [key, driver] : m_externalContactWrenches)
    {
        list.push(driver.polyDriverDescriptor.poly.get(), key.c_str());
    }

    // required for the IMUs
    list.push(m_masRemapper.poly.get(), "mas_remapper");

    if (!m_sensorBridge.setDriversList(list))
    {
        BipedalLocomotion::log()->error("{} Unable to set the driver list.", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateSwingFootPlanner(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::instantiateSwingFootPlanner]";
    auto ptr = handler->getGroup("SWING_FOOT_PLANNER").lock();

    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Unable to find the group 'SWING_FOOT_PLANNER'.",
                                        errorPrefix);
        return false;
    }

    auto tmp = ptr->clone();
    // todo bug blf
    /* tmp->setParameter("sampling_time", m_dT); */
    if (!m_leftFootPlanner.initialize(tmp))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the left foot planner.",
                                        errorPrefix);
        return false;
    }

    if (!m_rightFootPlanner.initialize(tmp))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the right foot planner.",
                                        errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::updateFloatingBase()
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::updateFloatingBase]";

    // set the contact phase list in the fixed foot detector
    // this function assumes that the contact phase list is not empty
    m_fixedFootDetector.setContactPhaseList(m_input.contactPhaseList);

    if (!m_floatingBaseEstimator.setKinematics(m_currentJointPos, m_currentJointVel))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinematics in the base estimator.",
                                        logPrefix);
        return false;
    }

    if (!m_fixedFootDetector.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to update the fixed foot detector.", logPrefix);
        return false;
    }

    for (const auto& [key, foot] : m_fixedFootDetector.getOutput())
    {
        // TODO Please change the signature of setContactStatus
        const auto frameName = m_kinDynWithDesired->model().getFrameName(foot.index);

        if (!m_floatingBaseEstimator.setContactStatus(frameName,
                                                      foot.isActive,
                                                      foot.switchTime,
                                                      foot.lastUpdateTime))
        {
            BipedalLocomotion::log()->error("{} Unable to set the contact status in the base "
                                            "estimator.",
                                            logPrefix);
            return false;
        }
    }

    if (!m_floatingBaseEstimator.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to update the floating base estimator.",
                                        logPrefix);
        return false;
    }

    if (!m_floatingBaseEstimator
             .changeFixedFrame(m_fixedFootDetector.getFixedFoot().index,
                               m_fixedFootDetector.getFixedFoot().pose.quat(),
                               m_fixedFootDetector.getFixedFoot().pose.translation()))
    {
        BipedalLocomotion::log()->error("{} Unable to change the fixed frame in the base "
                                        "estimator.",
                                        logPrefix);
        return false;
    }

    // this is the floating base with the IMU
    // Populate the floating base estimator output
    CentroidalMPCWalking::BaseEstimatorFromFootIMUInput input;
    input.jointPositions = m_currentJointPos;
    input.jointVelocities = m_currentJointVel;

    if (m_firstIteration)
    {
        input.measuredRotation_L = m_fixedFootDetector.getFixedFoot().pose.quat();
        input.measuredRotation_R = m_fixedFootDetector.getFixedFoot().pose.quat();
        input.measuredAngularVelocity_L.setZero();
        input.measuredAngularVelocity_R.setZero();
    } else
    {
        input.measuredRotation_L = BipedalLocomotion::Conversions::toManifRot(
            m_linksWithIMU.at("left_foot").averageRotation);
        input.measuredRotation_R = BipedalLocomotion::Conversions::toManifRot(
            m_linksWithIMU.at("right_foot").averageRotation);

        input.measuredAngularVelocity_L = m_linksWithIMU.at("left_foot").averageAngularVelocity;
        input.measuredAngularVelocity_R = m_linksWithIMU.at("right_foot").averageAngularVelocity;
    }
    input.stanceFootPose = m_fixedFootDetector.getFixedFoot().pose;
    // input.stanceFootPoseDesired = m_fixedFootDetector.getFixedFoot().pose;

    // check if it is the left of right
    const auto fixedFrameName
        = m_kinDynWithDesired->model().getFrameName(m_fixedFootDetector.getFixedFoot().index);

    input.isLeftStance = fixedFrameName == "l_sole";
    input.isRightStance = fixedFrameName == "r_sole";

    if (!m_baseEstimatorFromFootIMU.setInput(input))
    {
        BipedalLocomotion::log()->error("{} Unable to set the input in the base estimator with "
                                        "IMU.",
                                        logPrefix);
        return false;
    }

    if (!m_baseEstimatorFromFootIMU.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to update the base estimator with IMU.",
                                        logPrefix);
        return false;
    }

    m_baseTransform = m_floatingBaseEstimator.getOutput().basePose;
    m_baseTransformWithIMU = m_baseEstimatorFromFootIMU.getOutput().basePose;
    m_baseVelocity = m_floatingBaseEstimator.getOutput().baseTwist;
    m_baseVelocityWithIMU = m_baseEstimatorFromFootIMU.getOutput().baseVelocity;

    return true;
}

bool WholeBodyQPBlock::createPolydriver(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::createPolydriver]";

    std::string name;
    if (!handler->getParameter("name", name))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", errorPrefix);
        return false;
    }

    auto ptr = handler->getGroup("ROBOT_INTERFACE").lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Robot interface options is empty.", errorPrefix);
        return false;
    }

    auto tmp = ptr->clone();
    tmp->setParameter("local_prefix", name);

    std::vector<std::string> jointLists, jointsToBeRemoved;
    if (!ptr->getParameter("joints_list", jointLists))
    {
        BipedalLocomotion::log()->error("{} Unable to find the joint list.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("joints_to_be_removed", jointsToBeRemoved))
    {
        BipedalLocomotion::log()->error("{} Unable to find the joint to be removed.", errorPrefix);
        return false;
    }

    // remove the joints from the joint list keeping the order
    for (const auto& joint : jointsToBeRemoved)
    {
        jointLists.erase(std::remove(jointLists.begin(), jointLists.end(), joint),
                         jointLists.end());
    }

    tmp->setParameter("joints_list", jointLists);

    m_controlBoard = BipedalLocomotion::RobotInterface::constructRemoteControlBoardRemapper(tmp);

    if (!m_controlBoard.isValid())
    {
        BipedalLocomotion::log()->error("{} the robot polydriver has not been constructed.",
                                        errorPrefix);
        return false;
    }

    return true;
}

BipedalLocomotion::RobotInterface::PolyDriverDescriptor
WholeBodyQPBlock::createContactWrenchDriver(std::weak_ptr<const IParametersHandler> handler,
                                            const std::string& local)
{
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        return BipedalLocomotion::RobotInterface::PolyDriverDescriptor();
    }
    auto tmp = ptr->clone();
    tmp->setParameter("local_prefix", local);
    return BipedalLocomotion::RobotInterface::constructGenericSensorClient(tmp);
}

bool WholeBodyQPBlock::createAllContactWrenchesDriver(
    std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::createGenericSensorClient]";

    std::string localPrefix;
    if (!handler->getParameter("name", localPrefix))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", errorPrefix);
        return false;
    }

    auto ptr = handler->getGroup("CONTACT_WRENCHES").lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Robot interface options is empty.", errorPrefix);
        return false;
    }

    std::vector<std::string> contactWrenchClients;
    if (!ptr->getParameter("left_contact_wrenches_group", contactWrenchClients))
    {
        BipedalLocomotion::log()->error("{} Unable to find the left contact wrench group.",
                                        errorPrefix);
        return false;
    }

    for (const auto& wrench : contactWrenchClients)
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor descriptor
            = this->createContactWrenchDriver(ptr->getGroup(wrench), localPrefix);

        if (!descriptor.isValid())
        {
            BipedalLocomotion::log()->error("{} The generic sensor client for the wrench named {} "
                                            "cannot be opened.",
                                            errorPrefix,
                                            wrench);
            return false;
        }

        m_leftFootContacWrenches[descriptor.key].polyDriverDescriptor = descriptor;
    }

    if (!ptr->getParameter("right_contact_wrenches_group", contactWrenchClients))
    {
        BipedalLocomotion::log()->error("{} Unable to find the right contact wrench group.",
                                        errorPrefix);
        return false;
    }

    for (const auto& wrench : contactWrenchClients)
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor descriptor
            = this->createContactWrenchDriver(ptr->getGroup(wrench), localPrefix);

        if (!descriptor.isValid())
        {
            BipedalLocomotion::log()->error("{} The generic sensor client for the wrench named {} "
                                            "cannot be opened.",
                                            errorPrefix,
                                            wrench);
            return false;
        }

        m_rightFootContacWrenches[descriptor.key].polyDriverDescriptor = descriptor;
    }

    if (!ptr->getParameter("external_contact_wrenches_group", contactWrenchClients))
    {
        BipedalLocomotion::log()->error("{} Unable to find the right contact wrench group.",
                                        errorPrefix);
        return false;
    }

    for (const auto& wrench : contactWrenchClients)
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor descriptor
            = this->createContactWrenchDriver(ptr->getGroup(wrench), localPrefix);

        if (!descriptor.isValid())
        {
            BipedalLocomotion::log()->error("{} The generic sensor client for the wrench named {} "
                                            "cannot be opened.",
                                            errorPrefix,
                                            wrench);
            return false;
        }

        m_externalContactWrenches[descriptor.key].polyDriverDescriptor = descriptor;
    }

    return true;
}

bool WholeBodyQPBlock::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::initialize]";

    auto parametersHandler = handler.lock();

    if (parametersHandler == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("mpc_acts_as_planner", m_mpcActsAsPlanner))
    {
        BipedalLocomotion::log()->error("{} Unable to find the mpc_acts_as_planner parameter.",
                                        logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("filter_joint_vel", m_filterJointVel))
    {
        BipedalLocomotion::log()->error("{} Unable to find the filter_joint_vel parameter.",
                                        logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("use_imu_base_estimator_for_kindyn_measured",
                                         m_useIMUBaseEstimatorForKindynMeasured))
    {
        BipedalLocomotion::log()->error("{} Unable to find the "
                                        "use_imu_base_estimator_for_kindyn_measured parameter.",
                                        logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("use_imu_base_estimator_for_kindyn_desired",
                                         m_useIMUBaseEstimatorForKindynDesired))
    {
        BipedalLocomotion::log()->error("{} Unable to find the "
                                        "use_imu_base_estimator_for_kindyn_deasired parameter.",
                                        logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("use_imu_base_velocity_for_kindyn_desired",
                                         m_useIMUBaseVelocityForKindynDesired))
    {
        BipedalLocomotion::log()->error("{} Unable to find the use_imu_base_velocity_for_kindyn_"
                                        "desired parameter.",
                                        logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("use_imu_base_velocity_for_kindyn_measured",
                                         m_useIMUBaseVelocityForKindynMeasured))
    {
        BipedalLocomotion::log()->error("{} Unable to find the use_imu_base_velocity_for_kindyn_"
                                        "measured parameter.",
                                        logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("filter_gyroscope", m_filterGyroscope))
    {
        BipedalLocomotion::log()->error("{} Unable to find the filter_gyroscope.", logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("use_no_control_for_contact_foot",
                                         m_useNoControlForContactFoot))
    {
        BipedalLocomotion::log()->error("{} Unable to find the use_zero_velocity_for_contact_foot.",
                                        logPrefix);
        return false;
    }
    if (!parametersHandler->getParameter("enable_ankle_strategy", m_enableAnkleStrategy))
    {
        BipedalLocomotion::log()->error("{} Unable to find the enable_ankle_strategy.", logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("use_local_adjustment_joint_ankles",
                                         m_useLocalAdjustmentJointAnkles))
    {
        BipedalLocomotion::log()->error("{} Unable to find the use_local_adjustment_joint_ankles.",
                                        logPrefix);
        return false;
    }


    if (!parametersHandler->getParameter("enable_com_zmp_controller", m_enableCoMZMPController))
    {
        BipedalLocomotion::log()->error("{} Unable to find the enable_com_zmp_controller.",
                                        logPrefix);
        return false;
    }

    // m_useMeasuredBaseVelocityForIK
    if (!parametersHandler->getParameter("use_measured_base_velocity_for_ik",
                                         m_useMeasuredBaseVelocityForIK))
    {
        BipedalLocomotion::log()->error("{} Unable to find the use_measured_base_velocity_for_ik.",
                                        logPrefix);
        return false;
    }


    BipedalLocomotion::log()->info("mpc acts as planner: {}", m_mpcActsAsPlanner);
    BipedalLocomotion::log()->info("filter joint velocity: {}", m_filterJointVel);
    BipedalLocomotion::log()->info("use imu base estimator kindyn measured: {}",
                                   m_useIMUBaseEstimatorForKindynMeasured);
    BipedalLocomotion::log()->info("use imu base estimator kindyn desired: {}",
                                   m_useIMUBaseEstimatorForKindynDesired);
    BipedalLocomotion::log()->info("use imu base velocity for kindyn measured: {}",
                                   m_useIMUBaseVelocityForKindynMeasured);
    BipedalLocomotion::log()->info("use imu base velocity for kindyn desired: {}",
                                   m_useIMUBaseVelocityForKindynDesired);
    BipedalLocomotion::log()->info("filter gyroscope: {}", m_filterGyroscope);
    BipedalLocomotion::log()->info("use no control for contact foot: {}",
                                   m_useNoControlForContactFoot);
    BipedalLocomotion::log()->info("enable ankle strategy: {}", m_enableAnkleStrategy);
    BipedalLocomotion::log()->info("use local adjustment joint ankles: {}",
                                   m_useLocalAdjustmentJointAnkles);
    BipedalLocomotion::log()->info("enable com zmp controller: {}", m_enableCoMZMPController);
    BipedalLocomotion::log()->info("use measured base velocity for ik: {}",
                                   m_useMeasuredBaseVelocityForIK);

    m_flags.push_back(m_mpcActsAsPlanner ? 1 : 0);
    m_flags.push_back(m_filterJointVel ? 1 : 0);
    m_flags.push_back(m_useIMUBaseEstimatorForKindynMeasured ? 1 : 0);
    m_flags.push_back(m_useIMUBaseEstimatorForKindynDesired ? 1 : 0);
    m_flags.push_back(m_useIMUBaseVelocityForKindynMeasured ? 1 : 0);
    m_flags.push_back(m_useIMUBaseVelocityForKindynDesired ? 1 : 0);
    m_flags.push_back(m_filterGyroscope ? 1 : 0);
    m_flags.push_back(m_useNoControlForContactFoot ? 1 : 0);
    m_flags.push_back(m_enableAnkleStrategy ? 1 : 0);
    m_flags.push_back(m_useLocalAdjustmentJointAnkles ? 1 : 0);
    m_flags.push_back(m_enableCoMZMPController ? 1 : 0);
    m_flags.push_back(m_useMeasuredBaseVelocityForIK ? 1 : 0);

    auto wholeBodyRunnerHandler = parametersHandler->getGroup("WHOLE_BODY_RUNNER").lock();
    if (wholeBodyRunnerHandler == nullptr)
    {
        BipedalLocomotion::log()->error("{} Unable to find the whole body runner group.",
                                        logPrefix);
        return false;
    }

    // get the sampling time
    if (!wholeBodyRunnerHandler->getParameter("sampling_time", m_dT))
    {
        BipedalLocomotion::log()->error("{} Unable to find the sampling time.", logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{} Create the polydriver.", logPrefix);
    if (!this->createPolydriver(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to create the polydriver.", logPrefix);
        return false;
    }
    if (!this->createAllContactWrenchesDriver(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to create the contact wrench drivers.",
                                        logPrefix);
        return false;
    }

    // This sleep is required to let the robot interface to be ready
    BipedalLocomotion::clock().sleepFor(1s);

    auto ptrTmp = parametersHandler->getGroup("ROBOT_INTERFACE").lock();
    if (ptrTmp == nullptr)
    {
        BipedalLocomotion::log()->error("{} Unable to find the group 'ROBOT_INTERFACE'.",
                                        logPrefix);
        return false;
    }

    if (!ptrTmp->getParameter("joints_list", m_jointsList))
    {
        BipedalLocomotion::log()->error("{} Unable to find the joint list.", logPrefix);
        return false;
    }

    if (!this->createKinDyn(yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(
                                "model.urdf"),
                            m_jointsList))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the kinDyn.", logPrefix);
        return false;
    }

    if (!this->setBaseFrame("l_sole", "left_foot", m_kinDynWithDesired->model()))
    {
        BipedalLocomotion::log()->error("{} Unable to set the base frame left_foot.", logPrefix);
        return false;
    }

    if (!this->setBaseFrame("r_sole", "right_foot", m_kinDynWithDesired->model()))
    {
        BipedalLocomotion::log()->error("{} Unable to set the base frame right_foot.", logPrefix);
        return false;
    }

    if (!m_kinDynWithDesired->setFloatingBase("root_link")
        || !m_kinDynWithMeasured->setFloatingBase("root_link")
        || !m_kinDynWithRegularization->setFloatingBase("root_link"))
    {
        BipedalLocomotion::log()->error("{} Unable to set the floating base in the "
                                        "KinDynComputations objects.",
                                        logPrefix);
        return false;
    }

    if (!m_baseEstimatorFromFootIMU.setModel(m_kinDynWithMeasured->model()))
    {
        BipedalLocomotion::log()->error("{} Unable to set the model in the base estimator.",
                                        logPrefix);
        return false;
    }

    if (!m_baseEstimatorFromFootIMU.initialize(parametersHandler->getGroup("BASE_ESTIMATOR")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the base estimator.", logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{} Create the robot control helper.", logPrefix);
    if (!this->initializeRobotControl(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the robotControl interface.",
                                        logPrefix);
        return false;
    }

    if (!this->configureLinksWithIMU(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to configure the links with IMU.", logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{} Create the sensor bridge.", logPrefix);
    if (!this->instantiateSensorBridge(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the sensor bridge.", logPrefix);
        return false;
    }

    std::string name;
    if (!parametersHandler->getParameter("name", name))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", logPrefix);
        return false;
    }

    m_robotMass = m_kinDynWithMeasured->model().getTotalMass();

    if (!this->instantiateIK(parametersHandler->getGroup("IK")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the IK.", logPrefix);
        return false;
    }

    if (!m_IKandTasks.baseTask->setSetPoint(manif::SE3d::Identity()))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the base task.", logPrefix);
        return false;
    }

    if (!this->instantiateLeggedOdometry(parametersHandler,
                                         yarp::os::ResourceFinder::getResourceFinderSingleton()
                                             .findFileByName("model.urdf"),
                                         m_jointsList))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the legged odometry.", logPrefix);
        return false;
    }

    if (!m_CoMZMPController.initialize(parametersHandler->getGroup("COM_ZMP_CONTROLLER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the CoM-ZMP Controller.",
                                        logPrefix);
        return false;
    }

    if (!this->instantiateSwingFootPlanner(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the swing foot planner.",
                                        logPrefix);
        return false;
    }

    // resize the vectors
    const std::size_t numberOfJoints = m_robotControl.getJointList().size();
    std::cerr << "----------------------> " << numberOfJoints << std::endl;

    m_currentJointPos.resize(numberOfJoints);
    m_currentJointVel.resize(numberOfJoints);
    m_currentJointVel.setZero();
    if (m_filterJointVel)
    {
        if (!m_jointVelFilter.initialize(parametersHandler->getGroup("JOINT_VELOCITY_FILTER")))
        {
            BipedalLocomotion::log()->error("{} Unable to initialize the joint velocity filter.",
                                            logPrefix);
            return false;
        }
        m_jointVelFilter.reset(m_currentJointVel * 0);
    }

    m_currentJointPosWithoutNeck.resize(numberOfJoints);
    m_currentJointVelWithoutNeck.resize(numberOfJoints);
    m_currentJointPosWithoutNeck.setZero();

    m_currentJointPos.resize(m_jointsList.size());
    m_currentJointPos.setZero();
    m_currentJointVel.resize(m_jointsList.size());
    m_currentJointVel.setZero();
    m_desJointPosForRobot.resize(numberOfJoints);
    m_desJointPos.resize(m_jointsList.size());
    m_desJointVel.resize(m_jointsList.size());
    m_desJointVel.setZero();

    if (!m_sensorBridge.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to get the robot state.", logPrefix);
        return false;
    }

    // initialize the joint positions
    if (!m_sensorBridge.getJointPositions(m_currentJointPosWithoutNeck))
    {
        BipedalLocomotion::log()->error("{} Unable to get the joint positions.", logPrefix);
        return false;
    }

    // TODO remvoe me if the neck is working
    // m_currentJointPos.head<15>() = m_currentJointPosWithoutNeck.head<15>();
    // m_currentJointPos.tail<8>() = m_currentJointPosWithoutNeck.tail<8>();

    m_currentJointPos = m_currentJointPosWithoutNeck;

    m_desJointPos = m_currentJointPos;

    auto initializeDynamicalSystem
        = [logPrefix, this](auto& dynamicalSystem, const std::string& dynamicalSystemType) -> bool {
        if (!dynamicalSystem.integrator->setIntegrationStep(this->m_dT))
        {
            BipedalLocomotion::log()->error("{} Unable to set the integration step of {}.",
                                            logPrefix,
                                            dynamicalSystemType);
            return false;
        }
        if (!dynamicalSystem.integrator->setDynamicalSystem(dynamicalSystem.dynamics))
        {
            BipedalLocomotion::log()->error("{} Unable to set the dynamical system of {}.",
                                            logPrefix,
                                            dynamicalSystemType);
            return false;
        }
        return true;
    };

    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    m_floatingBaseSystem.dynamics = std::make_shared<LinearTimeInvariantSystem>();
    m_floatingBaseSystem.integrator = std::make_shared<RK4<LinearTimeInvariantSystem>>();
    m_floatingBaseSystem.dynamics
        ->setSystemMatrices(Eigen::MatrixXd::Zero(m_jointsList.size(), m_jointsList.size()),
                            Eigen::MatrixXd::Identity(m_jointsList.size(), m_jointsList.size()));
    if (!initializeDynamicalSystem(m_floatingBaseSystem, "Floating base system"))
    {
        return false;
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * m_jointsList.size(), 2 * m_jointsList.size());
    A.topRightCorner(m_jointsList.size(), m_jointsList.size()).setIdentity();

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2 * m_jointsList.size(), m_jointsList.size());
    B.bottomLeftCorner(m_jointsList.size(), m_jointsList.size()).setIdentity();


    m_centroidalSystem.dynamics = std::make_shared<CentroidalDynamics>();
    m_centroidalSystem.integrator = std::make_shared<RK4<CentroidalDynamics>>();
    if (!initializeDynamicalSystem(m_centroidalSystem, "Centroidal system"))
    {
        return false;
    }

    m_comSystem.dynamics = std::make_shared<LinearTimeInvariantSystem>();
    if (!m_comSystem.dynamics->setSystemMatrices(Eigen::Matrix2d::Zero(),
                                                 Eigen::Matrix2d::Identity()))
    {
        BipedalLocomotion::log()->error("{} Unable to set the system matrices of the CoM system.",
                                        logPrefix);
        return false;
    }
    m_comSystem.integrator = std::make_shared<RK4<LinearTimeInvariantSystem>>();
    if (!initializeDynamicalSystem(m_comSystem, "CoM system"))
    {
        return false;
    }

    // open logged data port
    if (!m_logDataServer.initialize(parametersHandler->getGroup("LOGGER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the log data server.", logPrefix);
        return false;
    }

    // find the index of the joints in m_jointsList
    m_leftAnklePitch.jointIndex
        = std::distance(m_jointsList.begin(),
                        std::find(m_jointsList.begin(), m_jointsList.end(), "l_ankle_pitch"));
    m_rightAnklePitch.jointIndex
        = std::distance(m_jointsList.begin(),
                        std::find(m_jointsList.begin(), m_jointsList.end(), "r_ankle_pitch"));
    m_leftAnkleRoll.jointIndex
        = std::distance(m_jointsList.begin(),
                        std::find(m_jointsList.begin(), m_jointsList.end(), "l_ankle_roll"));
    m_rightAnkleRoll.jointIndex
        = std::distance(m_jointsList.begin(),
                        std::find(m_jointsList.begin(), m_jointsList.end(), "r_ankle_roll"));

    BipedalLocomotion::log()->info("left ankle pitch joint index: {}", m_leftAnklePitch.jointIndex);
    BipedalLocomotion::log()->info("right ankle pitch joint index: {}",
                                   m_rightAnklePitch.jointIndex);
    BipedalLocomotion::log()->info("left ankle roll joint index: {}", m_leftAnkleRoll.jointIndex);
    BipedalLocomotion::log()->info("right ankle roll joint index: {}", m_rightAnkleRoll.jointIndex);
    BipedalLocomotion::log()->info("joint list size: {}", m_jointsList.size());

    // initialize the metadata
    m_logDataServer.populateMetadata("flags",
                                     {"mpc_acts_as_planner",
                                      "filter_joint_vel",
                                      "use_imu_base_estimator_for_kindyn_measured",
                                      "use_imu_base_estimator_for_kindyn_desired",
                                      "use_imu_base_velocity_for_kindyn_measured",
                                      "use_imu_base_velocity_for_kindyn_desired",
                                      "filter_gyroscope",
                                      "use_no_control_for_contact_foot",
                                      "enable_ankle_strategy",
                                      "use_local_adjustment_joint_ankles",
                                      "enable_com_zmp_controller",
                                      "use_measured_base_velocity_for_ik"});

    m_logDataServer.populateMetadata("com::position::measured", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::position::desired", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::position::given_as_feedback", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::position::mann", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::position::ik_input", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::position::mpc_output", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::velocity::mpc_output", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::velocity::ik_input", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::velocity::given_as_feedback", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::velocity::desired_by_ik", {"x", "y"});
    m_logDataServer.populateMetadata("com::velocity::desired", {"x", "y", "z"});
    m_logDataServer.populateMetadata("com::velocity::measured", {"x", "y", "z"});

    m_logDataServer.populateMetadata("com::position::mpc_output_integrated", {"x", "y", "z"});
    m_logDataServer.populateMetadata("angular_momentum::measured", {"x", "y", "z"});
    m_logDataServer.populateMetadata("angular_momentum::desired", {"x", "y", "z"});
    m_logDataServer.populateMetadata("angular_momentum::given_as_feedback", {"x", "y", "z"});
    m_logDataServer.populateMetadata("angular_momentum::mpc_output_integrated", {"x", "y", "z"});
    m_logDataServer.populateMetadata("angular_momentum::mpc_output", {"x", "y", "z"});
    m_logDataServer.populateMetadata("angular_momentum::mann", {"x", "y", "z"});

    m_logDataServer.populateMetadata("root_link::position::measured", {"x", "y", "z"});
    m_logDataServer.populateMetadata("root_link::position::imu_estimator", {"x", "y", "z"});
    m_logDataServer.populateMetadata("root_link::position::desired", {"x", "y", "z"});
    m_logDataServer.populateMetadata("root_link::orientation::measured", {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("root_link::orientation::imu_estimator",
                                     {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("fixed_foot::index", {"index"});
    m_logDataServer.populateMetadata("fixed_foot::is_left", {"is_left"});
    m_logDataServer.populateMetadata("fixed_foot::translation", {"x", "y", "z"});
    m_logDataServer.populateMetadata("fixed_foot::orientation", {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("left_foot::position::desired", {"x", "y", "z"});
    m_logDataServer.populateMetadata("left_foot::orientation::desired", {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("right_foot::position::desired", {"x", "y", "z"});
    m_logDataServer.populateMetadata("right_foot::orientation::desired", {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("left_foot::position::measured_desired", {"x", "y", "z"});
    m_logDataServer.populateMetadata("right_foot::position::measured_desired", {"x", "y", "z"});
    m_logDataServer.populateMetadata("left_foot::position::measured", {"x", "y", "z"});
    m_logDataServer.populateMetadata("right_foot::position::measured", {"x", "y", "z"});
    m_logDataServer.populateMetadata("left_foot::orientation::measured_desired",
                                     {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("right_foot::orientation::measured_desired",
                                     {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("left_foot::orientation::measured", {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("right_foot::orientation::measured", {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("left_foot::velocity::measured",
                                     {"x", "y", "z", "wx", "wy", "wz"});
    m_logDataServer.populateMetadata("left_foot::velocity::desired",
                                     {"x", "y", "z", "wx", "wy", "wz"});
    m_logDataServer.populateMetadata("left_foot::velocity::desired_ik",
                                     {"x", "y", "z", "wx", "wy", "wz"});
    m_logDataServer.populateMetadata("right_foot::velocity::measured",
                                     {"x", "y", "z", "wx", "wy", "wz"});
    m_logDataServer.populateMetadata("right_foot::velocity::desired",
                                     {"x", "y", "z", "wx", "wy", "wz"});
    m_logDataServer.populateMetadata("right_foot::velocity::desired_ik",
                                     {"x", "y", "z", "wx", "wy", "wz"});

    m_logDataServer.populateMetadata("left_foot::wrench::desired_mpc",
                                     {"fx", "fy", "fz", "mx", "my", "mz"});
    m_logDataServer.populateMetadata("right_foot::wrench::desired_mpc",
                                     {"fx", "fy", "fz", "mx", "my", "mz"});
    m_logDataServer.populateMetadata("base::velocity::desired_ik",
                                     {"x", "y", "z", "wx", "wy", "wz"});
    m_logDataServer.populateMetadata("base::velocity::measured", {"x", "y", "z", "wx", "wy", "wz"});
    m_logDataServer.populateMetadata("base::velocity::desired", {"x", "y", "z", "wx", "wy", "wz"});
    m_logDataServer.populateMetadata("base::position::without_imu", {"x", "y", "z"});
    m_logDataServer.populateMetadata("base::position::with_imu", {"x", "y", "z"});
    m_logDataServer.populateMetadata("base::orientation::without_imu", {"roll", "pitch", "yaw"});
    m_logDataServer.populateMetadata("base::orientation::with_imu", {"roll", "pitch", "yaw"});

    m_logDataServer.populateMetadata("computation_time::CentroidalMPC", {"time"});
    m_logDataServer.populateMetadata("computation_time::Adherent", {"time"});
    m_logDataServer.populateMetadata("computation_time::WholeBodyQP", {"time"});
    m_logDataServer.populateMetadata("computation_time::WholeBodyQP_send", {"time"});
    m_logDataServer.populateMetadata("zmp::desired", {"x", "y"});
    m_logDataServer.populateMetadata("zmp::measured", {"x", "y"});
    m_logDataServer.populateMetadata("zmp::local::left::desired", {"x", "y"});
    m_logDataServer.populateMetadata("zmp::local::right::desired", {"x", "y"});
    m_logDataServer.populateMetadata("zmp::local::left::measured", {"x", "y"});
    m_logDataServer.populateMetadata("zmp::local::right::measured", {"x", "y"});
    m_logDataServer.populateMetadata("zmp::local::left::is_defined", {"desired", "measured"});
    m_logDataServer.populateMetadata("zmp::local::right::is_defined", {"desired", "measured"});
    m_logDataServer.populateMetadata("external_wrench::filtered",
                                     {"fx", "fy", "fz", "mx", "my", "mz"});
    m_logDataServer.populateMetadata("external_wrench::raw", {"fx", "fy", "fz", "mx", "my", "mz"});
    m_logDataServer.populateMetadata("joints_state::positions::mann", m_jointsList);
    m_logDataServer.populateMetadata("joints_state::positions::desired", m_jointsList);
    m_logDataServer.populateMetadata("joints_state::velocities::measured", m_jointsList);
    m_logDataServer.populateMetadata("joints_state::velocities::desired", m_jointsList);
    m_logDataServer.populateMetadata("joints_state::velocities::regularization", m_jointsList);
    m_logDataServer
        .populateMetadata("joints_state::velocities::desired_feet",
                          {"l_ankle_pitch", "l_ankle_roll", "r_ankle_pitch", "r_ankle_roll"});
    m_logDataServer.populateMetadata("joypad::motion_direction", {"x", "y"});
    m_logDataServer.populateMetadata("joypad::facing_direction", {"x", "y"});

    for (const std::string& key : {"left_foot", "right_foot"})
    {
        m_logDataServer.populateMetadata("contact::" + key + "::position::desired",
                                         {"x", "y", "z"});
        m_logDataServer.populateMetadata("contact::" + key + "::orientation::desired",
                                         {"roll", "pitch", "yaw"});

        m_logDataServer.populateMetadata("contact::" + key + "::position::nominal",
                                         {"x", "y", "z"});
        m_logDataServer.populateMetadata("contact::" + key + "::orientation::nominal",
                                         {"roll", "pitch", "yaw"});

        for (const int cornerIndex : {0, 1, 2, 3})
        {
            m_logDataServer.populateMetadata("contact::" + key + "::corner"
                                                 + std::to_string(cornerIndex) + "::force",
                                             {"x", "y", "z"});
            m_logDataServer.populateMetadata("contact::" + key + "::corner"
                                                 + std::to_string(cornerIndex) + "::position",
                                             {"x", "y", "z"});
        }
    }

    for (const auto& [key, imu] : m_linksWithIMU)
    {
        m_logDataServer.populateMetadata("imu::" + key + "::average_angular_velocity",
                                         {"x", "y", "z"});
        m_logDataServer.populateMetadata("imu::" + key + "::original_average_angular_velocity",
                                         {"x", "y", "z"});
    }

    m_logDataServer.finalizeMetadata();

    m_leftFootPlanner.setTime(m_absoluteTime);
    m_rightFootPlanner.setTime(m_absoluteTime);

    // switch the control mode to position direct mode
    if (!m_robotControl.setControlMode(
            BipedalLocomotion::RobotInterface::IRobotControl::ControlMode::PositionDirect))
    {
        BipedalLocomotion::log()->error("{} Unable to switch the control mode to position direct.",
                                        logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{} The WholeBodyQPBlock has been configured.", logPrefix);

    return true;
}

const WholeBodyQPBlock::Output& WholeBodyQPBlock::getOutput() const
{
    return m_output;
}

bool WholeBodyQPBlock::setInput(const Input& input)
{

    /*     auto convertToMilliSeconds = [](const std::chrono::nanoseconds& time) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(time);
        };

        BipedalLocomotion::log()->info("[WholeBodyQPBlock::setInput] Setting the input Input time
       {}, " "absolute time {}", convertToMilliSeconds(input.currentTime),
                                       convertToMilliSeconds(m_absoluteTime)); */
    if (m_absoluteTime < input.currentTime)
    {
        return true;
    }
    m_input = input;
    return true;
}

bool WholeBodyQPBlock::evaluateZMP(Eigen::Ref<Eigen::Vector2d> zmp,
                                   Eigen::Ref<Eigen::Vector2d> localLeft,
                                   Eigen::Ref<Eigen::Vector2d> localRight,
                                   bool& isLocalLeftDefined,
                                   bool& isLocalRightDefine)
{
    isLocalLeftDefined = isLocalRightDefine = false;
    using namespace BipedalLocomotion;
    Eigen::Vector3d zmpRight, zmpLeft;
    zmpLeft.setZero();
    zmpRight.setZero();
    double totalZ = 0;

    Math::Wrenchd leftWrench = Math::Wrenchd::Zero();
    for (const auto& [key, value] : m_leftFootContacWrenches)
    {
        leftWrench += value.wrench;
    }
    Math::Wrenchd rightWrench = Math::Wrenchd::Zero();
    for (const auto& [key, value] : m_rightFootContacWrenches)
    {
        rightWrench += value.wrench;
    }

    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;
    if (rightWrench.force()(2) < 10)
        zmpRightDefined = 0.0;
    else
    {
        zmpRight(0) = -rightWrench.torque()(1) / rightWrench.force()(2);
        zmpRight(1) = rightWrench.torque()(0) / rightWrench.force()(2);
        zmpRight(2) = 0.0;
        zmpRightDefined = 1.0;
        totalZ += rightWrench.force()(2);
        zmpRight.head<2>() = saturateLocalZMP(zmpRight.head<2>());

        isLocalRightDefine = true;
        localRight = zmpRight.head<2>();
    }

    if (leftWrench.force()(2) < 10)
        zmpLeftDefined = 0.0;
    else
    {
        zmpLeft(0) = -leftWrench.torque()(1) / leftWrench.force()(2);
        zmpLeft(1) = leftWrench.torque()(0) / leftWrench.force()(2);
        zmpLeft(2) = 0.0;
        zmpLeftDefined = 1.0;
        totalZ += leftWrench.force()(2);
        zmpLeft.head<2>() = saturateLocalZMP(zmpLeft.head<2>());

        isLocalLeftDefined = true;
        localLeft = zmpLeft.head<2>();
    }

    if (totalZ < 0.1)
    {
        BipedalLocomotion::log()->error("[WholeBodyQPBlock::evaluateZMP] The total z-component of "
                                        "contact wrenches is too low.");
        return false;
    }

    manif::SE3d I_H_lf
        = BipedalLocomotion::Conversions::toManifPose(m_kinDynWithMeasured->getWorldTransform("l_"
                                                                                              "sol"
                                                                                              "e"));
    manif::SE3d I_H_rf
        = BipedalLocomotion::Conversions::toManifPose(m_kinDynWithMeasured->getWorldTransform("r_"
                                                                                              "sol"
                                                                                              "e"));

    zmpLeft = I_H_lf.act(zmpLeft);
    zmpRight = I_H_rf.act(zmpRight);

    // the global zmp is given by a weighted average
    zmp = ((leftWrench.force()(2) * zmpLeftDefined) / totalZ) * zmpLeft.head<2>()
          + ((rightWrench.force()(2) * zmpRightDefined) / totalZ) * zmpRight.head<2>();

    return true;
}

bool WholeBodyQPBlock::computeDesiredZMP(
    const std::map<std::string, BipedalLocomotion::Contacts::DiscreteGeometryContact>& contacts,
    Eigen::Ref<Eigen::Vector2d> zmp,
    Eigen::Ref<Eigen::Vector2d> localLeft,
    Eigen::Ref<Eigen::Vector2d> localRight,
    bool& isLocalLeftDefined,
    bool& isLocalRightDefine)
{
    isLocalLeftDefined = isLocalRightDefine = false;
    double totalZ = 0;
    zmp.setZero();

    BipedalLocomotion::Planners::SwingFootPlanner* planner;

    if (contacts.size() == 0)
    {
        BipedalLocomotion::log()->error("[WholeBodyQPBlock::computeDesiredZMP] No contact is "
                                        "provided.");
        return false;
    }

    for (const auto& [key, contact] : contacts)
    {
        Eigen::Vector3d localZMP;
        localZMP.setZero();

        planner = (key == "left_foot") ? &m_leftFootPlanner : &m_rightFootPlanner;

        BipedalLocomotion::Math::Wrenchd totalWrench = BipedalLocomotion::Math::Wrenchd::Zero();
        for (const auto& corner : contact.corners)
        {
            totalWrench.force() += corner.force;
            totalWrench.torque()
                += corner.position.cross(contact.pose.asSO3().inverse().act(corner.force));
        }
        if (totalWrench.force()(2) > 0.001 && planner->getOutput().mixedVelocity.coeffs().isZero())
        {
            totalZ += totalWrench.force()(2);
            localZMP(0) = -totalWrench.torque()(1) / totalWrench.force()(2);
            localZMP(1) = totalWrench.torque()(0) / totalWrench.force()(2);
            localZMP(2) = 0.0;

            localZMP.head<2>() = saturateLocalZMP(localZMP.head<2>());

            // the wrench is already expressed in mixed we have just to translate it
            // if left
            if (key == "left_foot")
            {
                manif::SE3d temptransofrm = BipedalLocomotion::Conversions::toManifPose(
                    m_kinDynWithDesired->getWorldTransform("l_sole"));
                zmp += totalWrench.force()(2) * temptransofrm.act(localZMP).head<2>();
                isLocalLeftDefined = true;
                localLeft = localZMP.head<2>();
            }
            // if left
            else if (key == "right_foot")
            {
                manif::SE3d temptransofrm = BipedalLocomotion::Conversions::toManifPose(
                    m_kinDynWithDesired->getWorldTransform("r_sole"));
                zmp += totalWrench.force()(2) * temptransofrm.act(localZMP).head<2>();
                isLocalRightDefine = true;
                localRight = localZMP.head<2>();
            } else
            {
                BipedalLocomotion::log()->error("Problem in evaluated the desired zmp");
                return false;
            }
        }
    }

    if (!isLocalLeftDefined && !isLocalRightDefine)
    {
        BipedalLocomotion::log()->error("[WholeBodyQPBlock::computeDesiredZMP] locals zmp not "
                                        "defined.");
        return false;
    }

    zmp = zmp / totalZ;

    return true;
}

bool WholeBodyQPBlock::advance()
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::advance]";

    auto tic = std::chrono::steady_clock::now();

    bool shouldAdvance = false;
    m_output.isValid = false;
    m_output.currentTime = m_absoluteTime;

    // check if the block should advance
    // in this case we need the contact phase list in order to compute the base position and
    // evaluate the robot state
    if (m_input.contactPhaseList.size() == 0)
    {
        return true;
    }

    // from now one we assume that the contact phase list contains a set of contacts
    Eigen::Vector2d desiredZMP = Eigen::Vector2d::Zero();
    Eigen::Vector2d measuredZMP = Eigen::Vector2d::Zero();
    Eigen::Vector2d desiredLocalZMPLeft = Eigen::Vector2d::Zero();
    Eigen::Vector2d desiredLocalZMPRight = Eigen::Vector2d::Zero();
    Eigen::Vector2d localZMPLeft = Eigen::Vector2d::Zero();
    Eigen::Vector2d localZMPRight = Eigen::Vector2d::Zero();
    bool isDesiredLocalZMPLeftDefined, isDesiredLocalZMPRightDefined, isLocalZMPLeftDefined,
        isLocalZMPRightDefined;
    isDesiredLocalZMPLeftDefined = isDesiredLocalZMPRightDefined = isLocalZMPLeftDefined
        = isLocalZMPRightDefined = false;

    //////////// get the Feedback from the robot //////////
    //
    if (!m_sensorBridge.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to get the robot state.", errorPrefix);
        return false;
    }

    if (!m_sensorBridge.getJointPositions(m_currentJointPosWithoutNeck)
        || !m_sensorBridge.getJointVelocities(m_currentJointVelWithoutNeck))
    {
        BipedalLocomotion::log()->error("{} Unable to get the joint positions and velocities.",
                                        errorPrefix);
        return false;
    }

    if (m_filterJointVel)
    {
        m_jointVelFilter.setInput(m_currentJointVelWithoutNeck);
        if (!m_jointVelFilter.advance())
        {
            BipedalLocomotion::log()->error("{} Unable to filter the joint velocities.",
                                            errorPrefix);
            return false;
        }

        m_currentJointVelWithoutNeck = m_jointVelFilter.getOutput();
    }

    // TODO remove me if the neck is working
    // m_currentJointPos.head<15>() = m_currentJointPosWithoutNeck.head<15>();
    // m_currentJointPos.tail<8>() = m_currentJointPosWithoutNeck.tail<8>();

    // m_currentJointVel.head<15>() = m_currentJointVelWithoutNeck.head<15>();
    // m_currentJointVel.tail<8>() = m_currentJointVelWithoutNeck.tail<8>();

    m_currentJointPos = m_currentJointPosWithoutNeck;
    m_currentJointVel = m_currentJointVelWithoutNeck;


    // get the cartesian wrenches associated to the left foot
    for (auto& [key, value] : m_leftFootContacWrenches)
    {
        if (!m_sensorBridge.getCartesianWrench(key, value.wrench))
        {
            BipedalLocomotion::log()->error("{} Unable to get the left wrench named {}.",
                                            errorPrefix,
                                            key);
            return false;
        }
    }

    // get the cartesian wrenches associated to the right foot
    for (auto& [key, value] : m_rightFootContacWrenches)
    {
        if (!m_sensorBridge.getCartesianWrench(key, value.wrench))
        {
            BipedalLocomotion::log()->error("{} Unable to get the right wrench named {}.",
                                            errorPrefix,
                                            key);
            return false;
        }
    }

    Eigen::Vector3d rpyTemp;
    if (!m_firstIteration)
    {
        for (auto& [linkName, imu] : m_linksWithIMU)
        {
            std::vector<iDynTree::Rotation> tempRots;
            tempRots.reserve(imu.IMUs.size());
            tempRots.clear();
            imu.averageAngularVelocity.setZero();
            imu.originalAverageAngularVelocity.setZero();
            for (auto& [imuName, orientationData] : imu.IMUs)
            {
                if (!m_sensorBridge.getOrientationSensorMeasurement(orientationData.orientationName,
                                                                    rpyTemp))
                {
                    BipedalLocomotion::log()->error("{} Unable to get the orientation sensor "
                                                    "measurement",
                                                    errorPrefix);
                    return false;
                }

                if (!m_sensorBridge.getGyroscopeMeasure(orientationData.gyroName,
                                                        orientationData.angularVelocity))
                {
                    BipedalLocomotion::log()->error("{} Unable to get the gyroscope measurement",
                                                    errorPrefix);
                    return false;
                }

                Eigen::Vector3d temp = orientationData.angularVelocity;
                if (m_filterGyroscope)
                {
                    orientationData.angularVelocityFilter.setInput(orientationData.angularVelocity);
                    if (!orientationData.angularVelocityFilter.advance())
                    {
                        BipedalLocomotion::log()->error("{} Unable to filter the gyroscope "
                                                        "measurement",
                                                        errorPrefix);
                        return false;
                    }

                    orientationData.angularVelocity
                        = orientationData.angularVelocityFilter.getOutput();
                }
                orientationData.I_R_IMU
                    = orientationData.I_R_I_IMU
                      * iDynTree::Rotation::RPY(rpyTemp(0), rpyTemp(1), rpyTemp(2));
                orientationData.I_R_sole = orientationData.I_R_IMU * orientationData.IMU_R_sole;
                tempRots.push_back(orientationData.I_R_sole);

                imu.averageAngularVelocity
                    += iDynTree::toEigen(orientationData.IMU_R_sole.inverse())
                       * orientationData.angularVelocity;

                imu.originalAverageAngularVelocity
                    += iDynTree::toEigen(orientationData.IMU_R_sole.inverse()) * temp;
            }
            iDynTree::geodesicL2MeanRotation(tempRots, imu.averageRotation);
            imu.averageAngularVelocity /= imu.IMUs.size();
            imu.originalAverageAngularVelocity /= imu.IMUs.size();
        }
    }
    //
    ////////////////////////////////////////////////////// end of the feedback from the robot

    Eigen::Vector3d gravity;
    gravity.setZero();
    gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    // update the floating base
    if (!this->updateFloatingBase())
    {
        BipedalLocomotion::log()->error("{} Unable to update the floating base.", errorPrefix);
        return false;
    }

    /////// update kinDyn
    if (!m_kinDynWithMeasured->setRobotState(m_useIMUBaseEstimatorForKindynMeasured
                                                 ? m_baseTransformWithIMU.transform()
                                                 : m_baseTransform.transform(),
                                             m_currentJointPos,
                                             m_useIMUBaseVelocityForKindynMeasured
                                                 ? iDynTree::make_span(m_baseVelocityWithIMU.data(),
                                                                       manif::SE3d::Tangent::DoF)
                                                 : iDynTree::make_span(m_baseVelocity.data(),
                                                                       manif::SE3d::Tangent::DoF),
                                             m_currentJointVel,
                                             gravity))
    {
        BipedalLocomotion::log()->error("{} Unable to set the robot state in the kinDyn object.",
                                        errorPrefix);
        return false;
    }

    if (!m_kinDynWithDesired->setRobotState(m_useIMUBaseEstimatorForKindynDesired
                                                ? m_baseTransformWithIMU.transform()
                                                : m_baseTransform.transform(),
                                            m_desJointPos,
                                            m_useIMUBaseVelocityForKindynDesired
                                                ? iDynTree::make_span(m_baseVelocityWithIMU.data(),
                                                                      manif::SE3d::Tangent::DoF)
                                                : iDynTree::make_span(m_baseVelocity.data(),
                                                                      manif::SE3d::Tangent::DoF),
                                            m_desJointVel,
                                            gravity))
    {
        BipedalLocomotion::log()->error("{} Unable to set the robot state in the kinDyn object.",
                                        errorPrefix);
        return false;
    }

    if (!m_kinDynJointsDesiredBaseMeasured
             ->setRobotState(m_baseTransformWithIMU.transform(),
                             m_desJointPos,
                             m_useIMUBaseVelocityForKindynMeasured
                                 ? iDynTree::make_span(m_baseVelocityWithIMU.data(),
                                                       manif::SE3d::Tangent::DoF)
                                 : iDynTree::make_span(m_baseVelocity.data(),
                                                       manif::SE3d::Tangent::DoF),
                             m_desJointVel,
                             gravity))
    {
        BipedalLocomotion::log()->error("{} Unable to set the robot state in the kinDyn object.",
                                        errorPrefix);
        return false;
    }


    if (m_firstIteration)
    {
        // if it is the first iteration we need to initialize the orientation of the IMUs
        for (auto& [linkName, imu] : m_linksWithIMU)
        {
            // Get the initial rotation matrix between the sole frame and the Inertial frame.
            // For the first stance foot, this matrix is the identity matrix.
            imu.soleInit_R_I = m_kinDynWithMeasured->getWorldTransform(imu.soleFrameIndex)
                                   .getRotation()
                                   .inverse();

            for (auto& [imuName, orientationData] : imu.IMUs)
            {
                if (!m_sensorBridge.getOrientationSensorMeasurement(orientationData.orientationName,
                                                                    rpyTemp))
                {
                    BipedalLocomotion::log()->error("{} Unable to get the orientation sensor "
                                                    "measurement",
                                                    errorPrefix);
                    return false;
                }
                orientationData.I_R_I_IMU
                    = (iDynTree::Rotation::RPY(rpyTemp(0), rpyTemp(1), rpyTemp(2))
                       * orientationData.IMU_R_sole * imu.soleInit_R_I)
                          .inverse();
            }
        }
    }

    // we get the upper part of the robot
    if (m_firstIteration)
    {
        m_jointPosRegularize = m_currentJointPos;
    }
    // TODO REMOVE ME IF THE NECK WORKS
    // m_jointPosRegularize[15] = 0.0;
    // m_jointPosRegularize[16] = 0.0;
    // m_jointPosRegularize[17] = 0.0;
    // m_jointPosRegularize.tail<14>() = m_input.regularizedJoints.tail<14>();

    m_jointPosRegularize = m_input.regularizedJoints;

    if (!m_kinDynWithRegularization->setRobotState(m_baseTransform.transform(),
                                                   m_jointPosRegularize,
                                                   iDynTree::make_span(m_baseVelocity.data(),
                                                                       manif::SE3d::Tangent::DoF),
                                                   m_desJointVel,
                                                   gravity))
    {
        BipedalLocomotion::log()->error("{} Unable to set the robot state in the kinDyn object.",
                                        errorPrefix);
        return false;
    }

    m_output.totalExternalWrench.setZero();

    // prepare the output for the MPC
    for (auto& [key, value] : m_externalContactWrenches)
    {
        if (!m_sensorBridge.getCartesianWrench(key, value.wrench))
        {
            BipedalLocomotion::log()->error("{} Unable to get the left wrench named {}.",
                                            errorPrefix,
                                            key);
            return false;
        }

        const manif::SO3d rotation = BipedalLocomotion::Conversions::toManifRot(
            m_kinDynWithMeasured->getWorldTransform("root_link").getRotation());
        m_output.totalExternalWrench += rotation * value.wrench;
    }

    m_output.totalExternalWrench.force() /= m_robotMass;
    m_output.totalExternalWrench.torque() /= m_robotMass;

    BipedalLocomotion::Math::Wrenchd externalWrenchRaw = m_output.totalExternalWrench;

    // TODO (Giulio): here we added a threshold probably it should be removed or better set as a
    // configuration parameter
    if (m_output.totalExternalWrench.force().norm() < 0.7)
    {
        m_output.totalExternalWrench.setZero();
    }

    // update the kinematics
    m_output.com = iDynTree::toEigen(m_kinDynWithDesired->getCenterOfMassPosition());
    m_output.dcom.setZero();
    m_output.angularMomentum.setZero();
    m_output.isValid = true;

    // this is the case in which the input provided by the MPC is not valid so we cannot proceed
    // further
    if (!m_input.isValid)
    {
        return true;
    }

    // // take the x and y components of the CoM
    // if (!m_comSystem.dynamics->setState({m_output.com.head<2>()}))
    // {
    //     BipedalLocomotion::log()->error("{} Unable to set the state for the CoM dynamics.",
    //                                     errorPrefix);
    //     return false;
    // }

    // if (!m_floatingBaseSystem.dynamics->setState({m_currentJointPos}))
    // {
    //     BipedalLocomotion::log()->error("{} Unable to set the state for the floating base "
    //                                     "dynamics.",
    //                                     errorPrefix);
    //     return false;
    // }

    // if this is the first iteration we need to initialize some quantities
    if (m_firstIteration)
    {
        // the mass is assumed to be 1
        if (!m_centroidalSystem.dynamics->setState(
                {m_output.com, m_output.dcom, m_output.angularMomentum / m_robotMass}))
        {
            BipedalLocomotion::log()->error("{} Unable to set the state for the centroidal "
                                            "dynamics.",
                                            errorPrefix);
            return false;
        }

        // take the x and y components of the CoM
        if (!m_comSystem.dynamics->setState({m_output.com.head<2>()}))
        {
            BipedalLocomotion::log()->error("{} Unable to set the state for the CoM dynamics.",
                                            errorPrefix);
            return false;
        }

        if (!m_floatingBaseSystem.dynamics->setState({m_currentJointPos}))
        {
            BipedalLocomotion::log()->error("{} Unable to set the state for the floating base "
                                            "dynamics.",
                                            errorPrefix);
            return false;
        }

        // TODO this can be provided by MANN
        if (!m_IKandTasks.regularizationTask->setSetPoint(m_currentJointPos))
        {
            BipedalLocomotion::log()->error("{} Unable to set the set point for the "
                                            "regularization task.",
                                            errorPrefix);
            return false;
        }

        if (m_IKandTasks.angularMomentumTask != nullptr)
        {
            if (!m_IKandTasks.angularMomentumTask->setSetPoint(Eigen::Vector3d::Zero()))
            {
                BipedalLocomotion::log()->error("{} Unable to initialize the angular momentum "
                                                "task.",
                                                errorPrefix);
                return false;
            }
        }

        m_leftAnklePitch.desired = m_currentJointPos(m_leftAnklePitch.jointIndex);
        m_rightAnklePitch.desired = m_currentJointPos(m_rightAnklePitch.jointIndex);
        m_leftAnkleRoll.desired = m_currentJointPos(m_leftAnkleRoll.jointIndex);
        m_rightAnkleRoll.desired = m_currentJointPos(m_rightAnkleRoll.jointIndex);

        m_rootLinkOffset
            = iDynTree::toEigen(m_kinDynWithMeasured->getWorldTransform("root_link").getPosition())
              - m_output.com;

        m_firstIteration = false;
    }

    if (m_mpcActsAsPlanner)
    {
        // the input is now valid so we can update the centroidal dynamics
        if (!m_centroidalSystem.dynamics->setControlInput(
                {m_input.controllerOutput.contacts, m_output.totalExternalWrench}))
        {
            BipedalLocomotion::log()->error("{} Unable to set the control input for the centroidal "
                                            "dynamics.",
                                            errorPrefix);
            return false;
        }
    }
    if (!m_leftFootPlanner.setContactList(m_input.contactPhaseList.lists().at("left_foot")))
    {
        BipedalLocomotion::log()->error("{} Unable to set the contact list for the left foot "
                                        "planner.",
                                        errorPrefix);
        return false;
    }
    if (!m_rightFootPlanner.setContactList(m_input.contactPhaseList.lists().at("right_foot")))
    {
        BipedalLocomotion::log()->error("{} Unable to set the contact list for the right foot "
                                        "planner.",
                                        errorPrefix);
        return false;
    }

    // advance the feet planners
    if (!m_leftFootPlanner.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to advance the left foot planner.", errorPrefix);
        return false;
    }

    if (!m_rightFootPlanner.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to advance the right foot planner.",
                                        errorPrefix);
        return false;
    }

    if (!this->computeDesiredZMP(m_input.controllerOutput.contacts,
                                 desiredZMP,
                                 desiredLocalZMPLeft,
                                 desiredLocalZMPRight,
                                 isDesiredLocalZMPLeftDefined,
                                 isDesiredLocalZMPRightDefined))
    {
        BipedalLocomotion::log()->error("{} Unable to compute the desired zmp.", errorPrefix);
        return false;
    }

    if (!this->evaluateZMP(measuredZMP,
                           localZMPLeft,
                           localZMPRight,
                           isLocalZMPLeftDefined,
                           isLocalZMPRightDefined))
    {
        BipedalLocomotion::log()->error("{} Unable to evaluate the measured zmp.", errorPrefix);
        return false;
    }
    const auto fixedFrameName
        = m_kinDynWithDesired->model().getFrameName(m_fixedFootDetector.getFixedFoot().index);

    std::array<double, 4> jointsVelocityDesiredFeet{0.0, 0.0, 0.0, 0.0};

    if (m_enableAnkleStrategy)
    {
        auto setState = [errorPrefix](auto& weight, const std::string& state) {
            // no need to set the state if the weight is a nullptr
            if (weight == nullptr)
            {
                return true;
            }
            if (!weight->setState(state))
            {
                BipedalLocomotion::log()->error("{} Unable to set the state for the task.",
                                                errorPrefix);
                return false;
            }
            return true;
        };

        // check if we are in ds ss_left or ss_right
        if (m_leftFootPlanner.getOutput().isInContact && m_rightFootPlanner.getOutput().isInContact)
        {
            if (!setState(m_IKandTasks.leftFootWeight, "ds")
                || !setState(m_IKandTasks.rightFootWeight, "ds")
                || !setState(m_IKandTasks.jointRegularizationWeight, "ds"))
            {
                BipedalLocomotion::log()->error("{} Unable to set the state ds ", errorPrefix);
                return false;
            }
        } else if (m_leftFootPlanner.getOutput().isInContact)
        {
            if (!setState(m_IKandTasks.leftFootWeight, "ss_left")
                || !setState(m_IKandTasks.rightFootWeight, "ss_left")
                || !setState(m_IKandTasks.jointRegularizationWeight, "ss_left"))
            {
                BipedalLocomotion::log()->error("{} Unable to set the state for the foot weight ",
                                                errorPrefix);
                return false;
            }
        } else if (m_rightFootPlanner.getOutput().isInContact)
        {
            if (!setState(m_IKandTasks.leftFootWeight, "ss_right")
                || !setState(m_IKandTasks.rightFootWeight, "ss_right")
                || !setState(m_IKandTasks.jointRegularizationWeight, "ss_right"))
            {
                BipedalLocomotion::log()->error("{} Unable to set the state for the ss_right ",

                                                errorPrefix);
                return false;
            }
        } else
        {
            BipedalLocomotion::log()->error("{} Unable to set the state for the foot weight task.",
                                            errorPrefix);
            return false;
        }

        if (isDesiredLocalZMPLeftDefined && isLocalZMPLeftDefined)
        {
            const Eigen::Vector2d error = desiredLocalZMPLeft - localZMPLeft;
            // TODO check me

            // dq = Kp * (q_des - q)
            // q_des  = kp1 * (zmp_des - zmp) + q
            m_jointPosRegularize(m_leftAnklePitch.jointIndex)
                = m_leftAnklePitch.admittanceGain * error(0)
                  + m_desJointPos(m_leftAnklePitch.jointIndex);
            m_jointPosRegularize(m_leftAnkleRoll.jointIndex)
                = m_desJointPos(m_leftAnkleRoll.jointIndex)
                  - m_leftAnkleRoll.admittanceGain * error(1);

            jointsVelocityDesiredFeet[0] = m_leftAnklePitch.admittanceGain * error(0);
            jointsVelocityDesiredFeet[1] = -m_leftAnkleRoll.admittanceGain * error(1);
        } else
        {
            m_jointPosRegularize(m_leftAnklePitch.jointIndex) = m_leftAnklePitch.desired;
            m_jointPosRegularize(m_leftAnkleRoll.jointIndex) = m_leftAnkleRoll.desired;
            jointsVelocityDesiredFeet[0] = 0.0;
            jointsVelocityDesiredFeet[1] = 0.0;
        }

        if (isDesiredLocalZMPRightDefined && isLocalZMPRightDefined)
        {
            const Eigen::Vector2d error = desiredLocalZMPRight - localZMPRight;
            m_jointPosRegularize(m_rightAnklePitch.jointIndex)
                = m_rightAnklePitch.admittanceGain * error(0)
                  + m_desJointPos(m_rightAnklePitch.jointIndex);
            m_jointPosRegularize(m_rightAnkleRoll.jointIndex)
                = m_desJointPos(m_rightAnkleRoll.jointIndex)
                  + m_rightAnkleRoll.admittanceGain * error(1);
            jointsVelocityDesiredFeet[2] = m_rightAnklePitch.admittanceGain * error(0);
            jointsVelocityDesiredFeet[3] = m_rightAnkleRoll.admittanceGain * error(1);
        } else
        {
            m_jointPosRegularize(m_rightAnklePitch.jointIndex) = m_rightAnklePitch.desired;
            m_jointPosRegularize(m_rightAnkleRoll.jointIndex) = m_rightAnkleRoll.desired;
            jointsVelocityDesiredFeet[2] = 0.0;
            jointsVelocityDesiredFeet[3] = 0.0;
        }
    }

    if (m_useNoControlForContactFoot)
    {
        if (fixedFrameName == "l_sole")
        {
            m_IKandTasks.leftFootTask->setTaskControllerMode(
                BipedalLocomotion::IK::SE3Task::Mode::Disable);
            m_IKandTasks.rightFootTask->setTaskControllerMode(
                BipedalLocomotion::IK::SE3Task::Mode::Enable);
        } else if (fixedFrameName == "r_sole")
        {
            m_IKandTasks.leftFootTask->setTaskControllerMode(
                BipedalLocomotion::IK::SE3Task::Mode::Enable);
            m_IKandTasks.rightFootTask->setTaskControllerMode(
                BipedalLocomotion::IK::SE3Task::Mode::Disable);
        } else
        {
            BipedalLocomotion::log()->error("{} The fixed frame is not a foot.", errorPrefix);
            return false;
        }
    }

    m_IKandTasks.leftFootTask->setFeedback(m_baseEstimatorFromFootIMU.getOutput().footPose_L);
    m_IKandTasks.rightFootTask->setFeedback(m_baseEstimatorFromFootIMU.getOutput().footPose_R);
    m_IKandTasks.comTask->setFeedback( //
        iDynTree::toEigen(m_kinDynWithMeasured->getCenterOfMassPosition()));

    // if (!m_leftFootPlanner.getOutput().mixedVelocity.coeffs().isZero())
    // {
    //     m_IKandTasks.leftFootTask->setTaskControllerMode(
    //         BipedalLocomotion::IK::SE3Task::Mode::Enable);
    // } else
    // {
    //     m_IKandTasks.leftFootTask->setTaskControllerMode(
    //         BipedalLocomotion::IK::SE3Task::Mode::Disable);
    // }

    // if (!m_rightFootPlanner.getOutput().mixedVelocity.coeffs().isZero())
    // {
    //     m_IKandTasks.rightFootTask->setTaskControllerMode(
    //         BipedalLocomotion::IK::SE3Task::Mode::Enable);
    // } else
    // {
    //     m_IKandTasks.rightFootTask->setTaskControllerMode(
    //         BipedalLocomotion::IK::SE3Task::Mode::Disable);
    // }

    // ZMP-COM controller
    if (m_mpcActsAsPlanner)
    {
        if (!m_centroidalSystem.integrator->integrate(0s, m_dT))
        {
            BipedalLocomotion::log()->error("{} Unable to integrate the centroidal dynamics.",
                                            errorPrefix);
            return false;
        }
    }
    using namespace BipedalLocomotion::GenericContainer::literals;
    Eigen::Vector3d comdes
        = m_mpcActsAsPlanner ? m_centroidalSystem.dynamics->getState().get_from_hash<"com_pos"_h>()
                             : m_input.controllerOutput.comTrajectory[1];
    Eigen::Vector3d dcomdes
        = m_mpcActsAsPlanner ? m_centroidalSystem.dynamics->getState().get_from_hash<"com_vel"_h>()
                             : m_input.controllerOutput.comVelocityTrajectory[1];

    Eigen::Vector3d angularMomentumDesired
        = m_mpcActsAsPlanner
              ? m_centroidalSystem.dynamics->getState().get_from_hash<"angular_momentum"_h>()
              : m_input.controllerOutput.angularMomentumTrajectory[1];

    Eigen::Vector3d ddcomdes;
    ddcomdes.setZero();
    ddcomdes(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    for (const auto& [key, contact] : m_input.controllerOutput.contacts)
    {
        for (const auto& corner : contact.corners)
        {
            ddcomdes += corner.force;
        }
    }

    // the X and Y position of the CoM is corrected by the CoM-ZMP controller
    m_CoMZMPController.setSetPoint(dcomdes.head<2>(), comdes.head<2>(), desiredZMP);

    // TODO (Giulio): the angle should be computed from the orientation of the base
    const double angle = 0;
    m_CoMZMPController
        .setFeedback(iDynTree::toEigen(m_kinDynWithMeasured->getCenterOfMassPosition()).head<2>(),
                     measuredZMP,
                     angle);
    if (!m_CoMZMPController.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to advance the ZMP-COM controller.",
                                        errorPrefix);
        return false;
    }

    if (!m_IKandTasks.regularizationTask->setSetPoint(m_jointPosRegularize))
    {
        BipedalLocomotion::log()->error("{} Unable to set the set point for the "
                                        "regularization task.",
                                        errorPrefix);
        return false;
    }

    if (m_IKandTasks.angularMomentumTask != nullptr)
    {
        if (!m_IKandTasks.angularMomentumTask->setSetPoint(angularMomentumDesired * m_robotMass))
        {
            BipedalLocomotion::log()->error("{} Unable to set the set point for the angular "
                                            "momentum task.",
                                            errorPrefix);
            return false;
        }
    }

    if (m_enableCoMZMPController)
    {
        dcomdes.head<2>() = m_CoMZMPController.getOutput();
        m_comSystem.dynamics->setControlInput({dcomdes.head<2>()});
        m_comSystem.integrator->integrate(0s, m_dT);
        comdes.head<2>() = std::get<0>(m_comSystem.integrator->getSolution());
    }

    if (m_useMeasuredBaseVelocityForIK)
    {
        if (!m_IKandTasks.baseTask->setSetPoint(BipedalLocomotion::Conversions::toManifPose(
                                                    m_kinDynWithMeasured->getWorldBaseTransform()),
                                                BipedalLocomotion::Conversions::toManifTwist(
                                                    m_kinDynWithMeasured->getBaseTwist())))
        {
            BipedalLocomotion::log()->error("{} Unable to set the set point for the base task.",
                                            errorPrefix);
            return false;
        }
    }

    if (!m_IKandTasks.comTask->setSetPoint(comdes, dcomdes))
    {
        BipedalLocomotion::log()->error("{} Unable to set the set point for the CoM task.",
                                        errorPrefix);
        return false;
    }

    Eigen::Vector3d rootlinkPos = comdes + m_rootLinkOffset;
    if (!m_IKandTasks.rootTask->setSetPoint(rootlinkPos, dcomdes))
    {
        BipedalLocomotion::log()->error("{} Unable to set the set point for the CoM task.",
                                        errorPrefix);
        return false;
    }

    if (!m_IKandTasks.leftFootTask->setSetPoint(m_leftFootPlanner.getOutput().transform,
                                                m_leftFootPlanner.getOutput().mixedVelocity))
    {
        BipedalLocomotion::log()->error("{} Unable to set the set point for the left foot task.",
                                        errorPrefix);
        return false;
    }

    if (!m_IKandTasks.rightFootTask->setSetPoint(m_rightFootPlanner.getOutput().transform,
                                                 m_rightFootPlanner.getOutput().mixedVelocity))
    {
        BipedalLocomotion::log()->error("{} Unable to set the set point for the right foot task.",
                                        errorPrefix);
        return false;
    }

    // to better stabilize the robot we add a task on the chest only for the yaw
    const double yaw = extactYawAngle(
        iDynTree::toEigen(m_kinDynWithRegularization->getWorldTransform("chest").getRotation()));
    if (!m_IKandTasks.chestTask->setSetPoint(manif::SO3d(Eigen::Quaterniond(
                                                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))),
                                             manif::SO3d::Tangent::Zero()))
    {
        BipedalLocomotion::log()->error("{} Unable to set the set point for the chest task.",
                                        errorPrefix);
        return false;
    }

    // evaluate the IK problem
    if (!m_IKandTasks.ikProblem.ik->advance())
    {
        BipedalLocomotion::log()->error("{} Unable to solve the IK problem", errorPrefix);
        return false;
    }

    Eigen::VectorXd tmpJointsVel = m_IKandTasks.ikProblem.ik->getOutput().jointVelocity;

    if (m_useLocalAdjustmentJointAnkles)
    {
        // enable the ankle strategy
        if (isLocalZMPLeftDefined && isDesiredLocalZMPLeftDefined)
        {
            Eigen::Vector2d error = desiredLocalZMPLeft - localZMPLeft;
            tmpJointsVel[m_leftAnklePitch.jointIndex] += m_leftAnklePitch.admittanceGain * error(0);
            tmpJointsVel[m_leftAnkleRoll.jointIndex] += -m_leftAnkleRoll.admittanceGain * error(1);
        }

        if (isLocalZMPRightDefined && isDesiredLocalZMPRightDefined)
        {
            Eigen::Vector2d error = desiredLocalZMPRight - localZMPRight;
            tmpJointsVel[m_rightAnklePitch.jointIndex]
                += m_rightAnklePitch.admittanceGain * error(0);
            tmpJointsVel[m_rightAnkleRoll.jointIndex] += m_rightAnkleRoll.admittanceGain * error(1);
        }
    }

    // integrate the joint velocity and the base velocity
    m_floatingBaseSystem.dynamics->setControlInput({tmpJointsVel});
    m_floatingBaseSystem.integrator->integrate(0s, m_dT);

    const auto& [jointPosition] = m_floatingBaseSystem.integrator->getSolution();
    m_desJointPos = jointPosition;    
    m_desJointVel = m_IKandTasks.ikProblem.ik->getOutput().jointVelocity;

    BipedalLocomotion::Math::Wrenchd leftWrench = BipedalLocomotion::Math::Wrenchd::Zero();
    BipedalLocomotion::Math::Wrenchd rightWrench = BipedalLocomotion::Math::Wrenchd::Zero();

    for (const auto& [key, contact] : m_input.controllerOutput.contacts)
    {
        BipedalLocomotion::Math::Wrenchd totalWrench = BipedalLocomotion::Math::Wrenchd::Zero();
        for (const auto& corner : contact.corners)
        {
            totalWrench.force() += corner.force;
            totalWrench.torque() += contact.pose.asSO3().act(
                corner.position.cross(contact.pose.asSO3().inverse().act(corner.force)));
        }

        totalWrench *= m_robotMass;

        if (key == "left_foot")
        {
            leftWrench = totalWrench;
        } else if (key == "right_foot")
        {
            rightWrench = totalWrench;
        }
    }

    m_desJointPosForRobot = m_desJointPos;    

    const auto controlMode = BipedalLocomotion::RobotInterface::IRobotControl::ControlMode::PositionDirect;
    if (!m_robotControl.setReferences(m_desJointPosForRobot,
                                      controlMode,
                                      m_currentJointPosWithoutNeck))
    {
        BipedalLocomotion::log()->error("{} Unable to set the reference", errorPrefix);
        return false;
    }

    m_output.com = m_mpcActsAsPlanner
                       ? m_centroidalSystem.dynamics->getState().get_from_hash<"com_pos"_h>()
                       : iDynTree::toEigen(m_kinDynWithMeasured->getCenterOfMassPosition());
    m_output.dcom = m_mpcActsAsPlanner
                        ? m_centroidalSystem.dynamics->getState().get_from_hash<"com_vel"_h>()
                        : iDynTree::toEigen(m_kinDynWithMeasured->getCenterOfMassVelocity());
    m_output.angularMomentum
        = m_mpcActsAsPlanner
              ? m_centroidalSystem.dynamics->getState().get_from_hash<"angular_momentum"_h>()
              : iDynTree::toEigen(
                    m_kinDynWithMeasured->getCentroidalTotalMomentum().getAngularVec3())
                    / m_robotMass;

    auto toc = std::chrono::steady_clock::now();
    Eigen::Matrix<double, 1, 1> computationTime{std::chrono::duration<double>(toc - tic).count()};

    std::vector<double> fixedFootIndex(1, m_fixedFootDetector.getFixedFoot().index);

    tic = std::chrono::steady_clock::now();

    auto toRPY = [](const manif::SO3d& mat) -> iDynTree::Vector3 {
        return BipedalLocomotion::Conversions::toiDynTreeRot(mat).asRPY();
    };

    m_logDataServer.prepareData();
    m_logDataServer.clearData();
    m_logDataServer.populateData("flags", m_flags);
    m_logDataServer.populateData("com::position::measured",
                                 m_kinDynWithMeasured->getCenterOfMassPosition());

    m_logDataServer.populateData("com::position::desired",
                                 m_kinDynWithDesired->getCenterOfMassPosition());

    m_logDataServer.populateData("com::position::given_as_feedback", m_output.com);
    m_logDataServer.populateData("com::position::mann", m_input.comMANN);
    m_logDataServer.populateData("com::position::ik_input", comdes);
    m_logDataServer
        .populateData("com::position::mpc_output_integrated",
                      m_centroidalSystem.dynamics->getState().get_from_hash<"com_pos"_h>());
    m_logDataServer.populateData("com::position::mpc_output",
                                 m_input.controllerOutput.comTrajectory[1]);

    m_logDataServer.populateData("com::velocity::mpc_output",
                                 m_input.controllerOutput.comVelocityTrajectory[1]);
    m_logDataServer.populateData("com::velocity::ik_input", dcomdes);
    m_logDataServer.populateData("com::velocity::given_as_feedback", m_output.dcom);
    m_logDataServer.populateData("com::velocity::desired_by_ik", m_IKandTasks.comTask->getB());
    m_logDataServer.populateData("com::velocity::desired",
                                 m_kinDynWithDesired->getCenterOfMassVelocity());
    m_logDataServer.populateData("com::velocity::measured",
                                 m_kinDynWithMeasured->getCenterOfMassVelocity());

    Eigen::Vector3d tmp1
        = iDynTree::toEigen(m_kinDynWithMeasured->getCentroidalTotalMomentum().getAngularVec3())
          / m_robotMass;
    m_logDataServer.populateData("angular_momentum::measured", tmp1);

    Eigen::Vector3d tmp2
        = iDynTree::toEigen(m_kinDynWithDesired->getCentroidalTotalMomentum().getAngularVec3())
          / m_robotMass;
    m_logDataServer.populateData("angular_momentum::desired", tmp2);

    m_logDataServer.populateData("angular_momentum::given_as_feedback", m_output.angularMomentum);
    m_logDataServer.populateData("angular_momentum::mpc_output_integrated",
                                 m_centroidalSystem.dynamics->getState()
                                     .get_from_hash<"angular_momentum"_h>());
    m_logDataServer.populateData("angular_momentum::mpc_output",
                                 m_input.controllerOutput.angularMomentumTrajectory[1]);
    m_logDataServer.populateData("angular_momentum::mann", m_input.angularMomentumMann);

    m_logDataServer.populateData("root_link::position::desired", rootlinkPos);
    m_logDataServer.populateData("root_link::position::measured",
                                 m_floatingBaseEstimator.getOutput().basePose.translation());
    m_logDataServer.populateData("root_link::orientation::measured",
                                 toRPY(m_floatingBaseEstimator.getOutput().basePose.quat()));
    m_logDataServer.populateData("root_link::position::imu_estimator",
                                 m_baseEstimatorFromFootIMU.getOutput().basePose.translation());
    m_logDataServer.populateData("root_link::orientation::imu_estimator",
                                 toRPY(m_baseEstimatorFromFootIMU.getOutput().basePose.quat()));

    std::array<double, 1> isLeft{
        m_kinDynWithDesired->model().getFrameName(m_fixedFootDetector.getFixedFoot().index)
                == "l_sole"
            ? 1.0
            : 0.0};
    m_logDataServer.populateData("fixed_foot::index", fixedFootIndex);
    m_logDataServer.populateData("fixed_foot::is_left", isLeft);
    m_logDataServer.populateData("fixed_foot::translation",
                                 m_fixedFootDetector.getFixedFoot().pose.translation());
    m_logDataServer.populateData("fixed_foot::orientation",
                                 toRPY(m_fixedFootDetector.getFixedFoot().pose.quat()));
    m_logDataServer.populateData("left_foot::position::desired",
                                 m_leftFootPlanner.getOutput().transform.translation());
    m_logDataServer.populateData("left_foot::orientation::desired",
                                 toRPY(m_leftFootPlanner.getOutput().transform.quat()));
    m_logDataServer.populateData("right_foot::position::desired",
                                 m_rightFootPlanner.getOutput().transform.translation());
    m_logDataServer.populateData("right_foot::orientation::desired",
                                 toRPY(m_rightFootPlanner.getOutput().transform.quat()));
    m_logDataServer.populateData("left_foot::position::measured_desired",
                                 m_kinDynWithDesired->getWorldTransform("l_sole").getPosition());
    m_logDataServer
        .populateData("left_foot::orientation::measured_desired",
                      m_kinDynWithDesired->getWorldTransform("l_sole").getRotation().asRPY());

    m_logDataServer.populateData("left_foot::position::measured",
                                 m_kinDynWithMeasured->getWorldTransform("l_sole").getPosition());
    m_logDataServer
        .populateData("left_foot::orientation::measured",
                      m_kinDynWithMeasured->getWorldTransform("l_sole").getRotation().asRPY());

    m_logDataServer.populateData("right_foot::position::measured_desired",
                                 m_kinDynWithDesired->getWorldTransform("r_sole").getPosition());
    m_logDataServer
        .populateData("right_foot::orientation::measured_desired",
                      m_kinDynWithDesired->getWorldTransform("r_sole").getRotation().asRPY());
    m_logDataServer.populateData("right_foot::position::measured",
                                 m_kinDynWithMeasured->getWorldTransform("r_sole").getPosition());
    m_logDataServer
        .populateData("right_foot::orientation::measured",
                      m_kinDynWithMeasured->getWorldTransform("r_sole").getRotation().asRPY());

    m_logDataServer.populateData("left_foot::velocity::measured",
                                 m_kinDynWithMeasured->getFrameVel("l_sole").asVector());
    m_logDataServer.populateData("left_foot::velocity::desired",
                                 m_kinDynWithDesired->getFrameVel("l_sole").asVector());
    m_logDataServer.populateData("left_foot::velocity::desired_ik",
                                 m_IKandTasks.leftFootTask->getB());
    m_logDataServer.populateData("right_foot::velocity::measured",
                                 m_kinDynWithMeasured->getFrameVel("r_sole").asVector());
    m_logDataServer.populateData("right_foot::velocity::desired",
                                 m_kinDynWithDesired->getFrameVel("r_sole").asVector());
    m_logDataServer.populateData("right_foot::velocity::desired_ik",
                                 m_IKandTasks.rightFootTask->getB());

    m_logDataServer.populateData("left_foot::wrench::desired_mpc", leftWrench);
    m_logDataServer.populateData("right_foot::wrench::desired_mpc", rightWrench);

    m_logDataServer
        .populateData("computation_time::CentroidalMPC",
                      std::array<double, 1>{
                          std::chrono::duration<double>(m_input.mpcComputationTime).count()});
    m_logDataServer
        .populateData("computation_time::Adherent",
                      std::array<double, 1>{
                          std::chrono::duration<double>(m_input.adherentComputationTime).count()});
    m_logDataServer.populateData("computation_time::WholeBodyQP", computationTime);
    m_logDataServer.populateData("zmp::desired", desiredZMP);
    m_logDataServer.populateData("zmp::measured", measuredZMP);

    Eigen::Vector3d tmpLocalZMPLeft = Eigen::Vector3d::Zero();
    tmpLocalZMPLeft.head<2>() = localZMPLeft;
    tmpLocalZMPLeft = BipedalLocomotion::Conversions::toManifPose( //
                          m_kinDynWithMeasured->getWorldTransform("l_sole"))
                          .act(tmpLocalZMPLeft);
    m_logDataServer.populateData("zmp::local::left::measured", tmpLocalZMPLeft.head<2>());

    tmpLocalZMPLeft.head<2>() = desiredLocalZMPLeft;
    tmpLocalZMPLeft = BipedalLocomotion::Conversions::toManifPose( //
                          m_kinDynWithMeasured->getWorldTransform("l_sole"))
                          .act(tmpLocalZMPLeft);
    m_logDataServer.populateData("zmp::local::left::desired", tmpLocalZMPLeft.head<2>());

    Eigen::Vector3d tmpLocalZMPRight = Eigen::Vector3d::Zero();
    tmpLocalZMPRight.head<2>() = localZMPRight;
    tmpLocalZMPRight = BipedalLocomotion::Conversions::toManifPose( //
                           m_kinDynWithMeasured->getWorldTransform("r_sole"))
                           .act(tmpLocalZMPRight);
    m_logDataServer.populateData("zmp::local::right::measured", tmpLocalZMPRight.head<2>());

    tmpLocalZMPRight.head<2>() = desiredLocalZMPRight;
    tmpLocalZMPRight = BipedalLocomotion::Conversions::toManifPose( //
                           m_kinDynWithMeasured->getWorldTransform("r_sole"))
                           .act(tmpLocalZMPRight);
    m_logDataServer.populateData("zmp::local::right::desired", tmpLocalZMPRight.head<2>());

    m_logDataServer.populateData("zmp::local::left::is_defined",
                                 std::array<double, 2>{isDesiredLocalZMPLeftDefined ? 1.0 : 0.0,
                                                       isLocalZMPLeftDefined ? 1.0 : 0.0});
    m_logDataServer.populateData("zmp::local::right::is_defined",
                                 std::array<double, 2>{isDesiredLocalZMPRightDefined ? 1.0 : 0.0,
                                                       isLocalZMPRightDefined ? 1.0 : 0.0});
    m_logDataServer.populateData("external_wrench::filtered", m_output.totalExternalWrench);
    m_logDataServer.populateData("external_wrench::raw", externalWrenchRaw);
    m_logDataServer.populateData("joints_state::positions::mann", m_jointPosRegularize);
    m_logDataServer.populateData("joints_state::positions::desired", m_desJointPos);
    m_logDataServer.populateData("joints_state::velocities::desired", m_desJointVel);
    m_logDataServer.populateData("joints_state::velocities::measured", m_currentJointVel);
    m_logDataServer.populateData("joints_state::velocities::regularization",
                                 m_IKandTasks.regularizationTask->getB());

    m_logDataServer.populateData("joints_state::velocities::desired_feet",
                                 jointsVelocityDesiredFeet);
    m_logDataServer.populateData("base::velocity::desired_ik",
                                 m_IKandTasks.ikProblem.ik->getOutput().baseVelocity.coeffs());
    m_logDataServer.populateData("base::velocity::measured",
                                 m_kinDynWithMeasured->getBaseTwist().asVector());
    m_logDataServer.populateData("base::velocity::desired",
                                 m_kinDynWithDesired->getBaseTwist().asVector());

    m_logDataServer.populateData("base::position::without_imu", m_baseTransform.translation());
    m_logDataServer.populateData("base::position::with_imu", m_baseTransformWithIMU.translation());
    m_logDataServer.populateData("base::orientation::without_imu", toRPY(m_baseTransform.quat()));
    m_logDataServer.populateData("base::orientation::with_imu",
                                 toRPY(m_baseTransformWithIMU.quat()));

    m_logDataServer.populateData("joypad::motion_direction", m_input.motionDirection);
    m_logDataServer.populateData("joypad::facing_direction", m_input.facingDirection);

    for (const auto& [key, contact] : m_input.controllerOutput.contacts)
    {
        m_logDataServer.populateData("contact::" + key + "::position::desired",
                                     contact.pose.translation());
        m_logDataServer.populateData("contact::" + key + "::orientation::desired",
                                     toRPY(contact.pose.quat()));
        int i = 0;
        for (const auto& corner : contact.corners)
        {
            m_logDataServer.populateData("contact::" + key + "::corner" + std::to_string(i)
                                             + "::force",
                                         corner.force);
            m_logDataServer.populateData("contact::" + key + "::corner" + std::to_string(i)
                                             + "::position",
                                         corner.position);
            i++;
        }
    }

    for (const auto& [key, contactList] : m_input.mannContactPhaseList.lists())
    {
        auto contact = contactList.getPresentContact(m_absoluteTime);
        if (contact == contactList.cend())
        {
            continue;
        }

        m_logDataServer.populateData("contact::" + key + "::position::nominal",
                                     contact->pose.translation());
        m_logDataServer.populateData("contact::" + key + "::orientation::nominal",
                                     toRPY(contact->pose.quat()));
    }

    for (const auto& [key, imu] : m_linksWithIMU)
    {
        m_logDataServer.populateData("imu::" + key + "::average_angular_velocity",
                                     imu.averageAngularVelocity);
        m_logDataServer.populateData("imu::" + key + "::original_average_angular_velocity",
                                     imu.originalAverageAngularVelocity);
    }

    toc = std::chrono::steady_clock::now();
    Eigen::Matrix<double, 1, 1> computationTimeSend{
        std::chrono::duration<double>(toc - tic).count()};
    m_logDataServer.populateData("computation_time::WholeBodyQP_send", computationTimeSend);

    m_logDataServer.sendData();

    // advance the time
    m_absoluteTime += m_dT;

    return true;
}

bool WholeBodyQPBlock::isOutputValid() const
{
    return true;
}

bool WholeBodyQPBlock::close()
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::close]";

    // switch the control mode to position direct mode
    if (!m_robotControl.setControlMode(
            BipedalLocomotion::RobotInterface::IRobotControl::ControlMode::Position))
    {
        BipedalLocomotion::log()->error("{} Unable to switch the control mode to position.",
                                        logPrefix);
        return false;
    }

    return true;
}

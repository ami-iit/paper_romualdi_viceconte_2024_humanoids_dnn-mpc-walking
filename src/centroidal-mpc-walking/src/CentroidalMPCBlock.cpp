/**
 * @file CentroidalMPCBlock.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <manif/manif.h>

#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/Contacts/ContactListJsonParser.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/ModelLoader.h>

#include <CentroidalMPCWalking/CentroidalMPCBlock.h>

#include <iDynTree/EigenHelpers.h>

#include <numeric>

using namespace CentroidalMPCWalking;
using namespace BipedalLocomotion::ParametersHandler;

double comz0mann = 0;

bool CentroidalMPCBlock::runMANNAsynchronously()
{
    constexpr auto logPrefix = "[CentroidalMPCBlock::runMANNAsynchronously]";
    auto tic = std::chrono::steady_clock::now();
    if (!m_generator.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to compute one step of the MANN generator.",
                                        logPrefix);
        return false;
    }
    auto toc = std::chrono::steady_clock::now();

    m_output.adherentComputationTime = toc - tic;

    return m_generator.isOutputValid();
}

bool updateContactPhaseList(const std::chrono::nanoseconds& currentTime,
                            const BipedalLocomotion::Contacts::ContactPhaseList& mannPhaseList,
                            const BipedalLocomotion::Contacts::ContactPhaseList& mpcPhaseList,
                            BipedalLocomotion::Contacts::ContactPhaseList& contactPhaseList)
{
    BipedalLocomotion::Contacts::ContactListMap contactListMap;
    using BipedalLocomotion::log;
    constexpr auto logPrefix = "[updateContactPhaseList]";

    using namespace std::chrono_literals;
    // to prevent the MPC to move the contact just before the contact switch we added a small buffer in the CentroidalMPC class
    std::chrono::nanoseconds adjustmentPreventionTime = 200ms;
    auto currentTimeAdjusted = currentTime + adjustmentPreventionTime;


    for (const auto& [name, contactList] : mannPhaseList.lists())
    {
        // get the index of the current contact in the mpc phase list
        const auto& mpcList = mpcPhaseList.lists().at(name);

        // get the index of the next contact in the mann phase list
        for (auto mannIt = contactList.getNextContact(currentTime); //
             mannIt != contactList.cend();
             ++mannIt)
        {
            // if the activation time of the contact is lower than equal to the time in the future
            // from which the MPC cannot change the contact anymore then
            // 1. we check if the MPC has a contact at the same time
            // 2. if the MPC has a contact at the same time we add the contact to the contact list where
            //    the activation / deactivation are taken from the MANN and the rest from the MPC
            auto contact = *mannIt;
            if (contact.activationTime <= currentTimeAdjusted)
            {
                auto mpcIt = mpcList.getActiveContact(currentTimeAdjusted);
                if (mpcIt == mpcList.cend())
                {
                    log()->error("{} Unable to find the contact of the {} foot at time {} in the MPC list.",
                                 logPrefix,
                                 name,
                                 std::chrono::duration_cast<std::chrono::milliseconds>(currentTimeAdjusted));
                    return false;
                }

                contact = *mpcIt;
                contact.activationTime = mannIt->activationTime;
                contact.deactivationTime = mannIt->deactivationTime;
            }


            if (!contactListMap[name].addContact(contact))
            {
                log()->error("{} Unable to add the contact to the contact list of the {} foot.",
                             logPrefix,
                             name);
                log()->error("This should never happen since the contact list of mann cannot be "
                             "inconsistent.");
                return false;
            }
        }


        // get the index of the current contact in the mpc phase list
        auto mpcPresentContact = mpcList.getActiveContact(currentTime);
        if (mpcPresentContact == mpcList.cend())
        {
            // nothing to do
            continue;
        }

        auto mannPresentContact = contactList.getActiveContact(currentTime);
        if (mannPresentContact == contactList.cend())
        {
            log()->error("{} Unable to find the current contact of the {} foot at time {}.",
                         logPrefix,
                         name,
                         std::chrono::duration_cast<std::chrono::milliseconds>(currentTime));
            return false;
        }

        auto contact = *mpcPresentContact;
        contact.activationTime = (*mannPresentContact).activationTime;
        contact.deactivationTime = (*mannPresentContact).deactivationTime;
        if (!contactListMap[name].addContact(contact))
        {
            log()->error("{} Unable to add the contact to the contact list of the {} foot. For "
                         "the current contact.",
                         logPrefix,
                         name);

            log()->error("mpc contact activation time {}, deactivation time {}",
                         std::chrono::duration_cast<std::chrono::milliseconds>(
                             mpcPresentContact->activationTime),
                         std::chrono::duration_cast<std::chrono::milliseconds>(
                             mpcPresentContact->deactivationTime));

            log()->error("mann contact activation time {}, deactivation time {}",
                         std::chrono::duration_cast<std::chrono::milliseconds>(
                             (*mannPresentContact).activationTime),
                         std::chrono::duration_cast<std::chrono::milliseconds>(
                             (*mannPresentContact).deactivationTime));

            log()->error("current time {}",
                         std::chrono::duration_cast<std::chrono::milliseconds>(currentTime));
            return false;
        }
    }

    contactPhaseList.setLists(contactListMap);

    return true;
}

bool CentroidalMPCBlock::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[CentroidalMPCBlock::initialize]";
    using BipedalLocomotion::log;
    using namespace std::chrono_literals;

    auto getParameter = [logPrefix](std::weak_ptr<const IParametersHandler> handler,
                                    const std::string& paramName,
                                    auto& param) -> bool {
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            log()->error("{} Invalid parameter handler.", logPrefix);
            return false;
        }

        if (!ptr->getParameter(paramName, param))
        {
            log()->error("{} Unable to find the parameter named {}.", logPrefix, paramName);
            return false;
        }

        return true;
    };

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    if (!m_controller.initialize(ptr->getGroup("CENTROIDAL_MPC")))
    {
        log()->error("{} Unable to initialize the MPC.", logPrefix);
        return false;
    }

    std::string modelPath
        = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName("model.urdf");
    BipedalLocomotion::log()->info("{} Model path: {}", logPrefix, modelPath);

    std::vector<std::string> jointsList;
    if (!getParameter(ptr->getGroup("MANN"), "joints_list", jointsList))
    {
        log()->error("{} Unable to get the parameter required by the MANN network.", logPrefix);
        return false;
    }

    if (!getParameter(ptr->getGroup("CENTROIDAL_MPC"), "sampling_time", m_dT))
    {
        log()->error("{} Unable to get the sampling_time.", logPrefix);
        return false;
    }

    iDynTree::ModelLoader ml;
    ml.loadReducedModelFromFile(modelPath, jointsList);
    if (!m_generator.setRobotModel(ml.model()))
    {
        log()->error("{} Unable to set the robot model for MANN.", logPrefix);
        return false;
    }

    m_robotMass = ml.model().getTotalMass();

    // find the onnx file
    auto tmpMANN = ptr->getGroup("MANN").lock();
    if (tmpMANN == nullptr)
    {
        log()->error("{} Unable to get the MANN group.", logPrefix);
        return false;
    }
    tmpMANN = tmpMANN->getGroup("MANN").lock();
    if (tmpMANN == nullptr)
    {
        log()->error("{} Unable to get the MANN group.", logPrefix);
        return false;
    }

    std::string onnxFileName;
    if (!tmpMANN->getParameter("onnx_model_name", onnxFileName))
    {
        log()->error("{} Unable to get the onnx file name.", logPrefix);
        return false;
    }
    std::string onnxFilePath
        = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(onnxFileName);

    tmpMANN->setParameter("onnx_model_path", onnxFilePath);

    if (!m_generator.initialize(ptr->getGroup("MANN")))
    {
        log()->error("{} Unable to initialize the MANN trajectory generator class.", logPrefix);
        return false;
    }

    if (!m_generatorInputBuilder.initialize(ptr->getGroup("MANN")))
    {
        log()->error("{} Unable to initialize the MANN trajectory generator input builder class.",
                     logPrefix);
        return false;
    }

    // if the previous went fine then the handler is not expired
    auto ptrMann = ptr->getGroup("MANN").lock();
    std::chrono::nanoseconds mannHorizon, mannSamplingTime;
    double slowDownFactor;
    if (!ptrMann->getParameter("time_horizon", mannHorizon)
        || !ptrMann->getParameter("sampling_time", mannSamplingTime)
        || !ptrMann->getParameter("slow_down_factor", slowDownFactor))
    {
        log()->error("{} Unable to get the time horizon or the sampling time.", logPrefix);
        return false;
    }
    std::size_t numberOfMannKnots = mannHorizon / mannSamplingTime;
    m_comFrequencyAdapter.inputTimeKnots.resize(numberOfMannKnots);
    m_comFrequencyAdapter.inputTimeKnotsAbsolute.resize(numberOfMannKnots);
    m_angularMomentumFrequencyAdapter.inputTimeKnots.resize(numberOfMannKnots);
    m_angularMomentumFrequencyAdapter.inputTimeKnotsAbsolute.resize(numberOfMannKnots);
    for (std::size_t i = 0; i < numberOfMannKnots; i++)
    {
        m_comFrequencyAdapter.inputTimeKnots[i] = i * mannSamplingTime;
        m_comFrequencyAdapter.inputTimeKnots[i] *= slowDownFactor;
        m_angularMomentumFrequencyAdapter.inputTimeKnots[i] = i * mannSamplingTime;
        m_angularMomentumFrequencyAdapter.inputTimeKnots[i] *= slowDownFactor;
    }

    auto ptrMPC = ptr->getGroup("CENTROIDAL_MPC").lock();
    std::chrono::nanoseconds mpcHorizon, mpcSamplingTime;
    if (!ptrMPC->getParameter("time_horizon", mpcHorizon)
        || !ptrMPC->getParameter("sampling_time", mpcSamplingTime))
    {
        log()->error("{} Unable to get the time horizon or the sampling time.", logPrefix);
        return false;
    }
    std::size_t numberOfMpcKnots = mpcHorizon / mpcSamplingTime + 1;
    m_comFrequencyAdapter.outputTimeKnots.resize(numberOfMpcKnots);
    m_angularMomentumFrequencyAdapter.outputTimeKnots.resize(numberOfMpcKnots);
    m_comFrequencyAdapter.outputPoints.resize(numberOfMpcKnots);
    m_comFrequencyAdapter.outputTimeKnotsAbsolute.resize(numberOfMpcKnots);
    m_angularMomentumFrequencyAdapter.outputPoints.resize(numberOfMpcKnots);
    m_angularMomentumFrequencyAdapter.outputTimeKnotsAbsolute.resize(numberOfMpcKnots);
    m_comFrequencyAdapter.dummy.resize(numberOfMpcKnots);
    m_angularMomentumFrequencyAdapter.dummy.resize(numberOfMpcKnots);

    for (std::size_t i = 0; i < numberOfMpcKnots; i++)
    {
        m_comFrequencyAdapter.outputTimeKnots[i] = i * mpcSamplingTime;
        m_angularMomentumFrequencyAdapter.outputTimeKnots[i] = i * mpcSamplingTime;
    }

    m_comFrequencyAdapter.spline.setInitialConditions(Eigen::Vector3d::Zero(),
                                                      Eigen::Vector3d::Zero());
    m_angularMomentumFrequencyAdapter.spline.setInitialConditions(Eigen::Vector3d::Zero(),
                                                                  Eigen::Vector3d::Zero());
    m_comFrequencyAdapter.spline.setFinalConditions(Eigen::Vector3d::Zero(),
                                                    Eigen::Vector3d::Zero());
    m_angularMomentumFrequencyAdapter.spline.setFinalConditions(Eigen::Vector3d::Zero(),
                                                                Eigen::Vector3d::Zero());

    m_mannCallingTime = std::chrono::nanoseconds(
        std::lcm(int(slowDownFactor) * mannSamplingTime.count(), mpcSamplingTime.count()));

    m_mergePointIndex = m_mannCallingTime / (int(slowDownFactor) * mannSamplingTime);
    log()->info("{} mann calling time {}",
                logPrefix,
                std::chrono::duration_cast<std::chrono::milliseconds>(m_mannCallingTime));
    log()->info("{} mann merge point index {}", logPrefix, m_mergePointIndex);

    // set the initial state of mann trajectory generator
    // clang-format off
    Eigen::VectorXd jointPositions(26);
    jointPositions <<
        -0.10914914922234864, 0.013321900684695305, 0.0641749643461214, -0.10257791368141178, -0.10022507712940709, -0.008216588774319855,// left leg
        -0.12268291054316265, 0.030634497603792124, 0.07615972729195111, -0.08458915163006389, -0.09374216923819316, 0.03547153929302758, // right leg
         0.15820784458809578, 0.0027573447757581046, -0.00487324344589554, // torso
        -0.00020607396841307649, -0.0024925787007575857, 0.044068009171592995, // neck
        -0.027139990021827265, 0.10001107590632177, -0.20205046715326178, 0.03895909848833218, // left arm
        -0.03078463156388759, 0.09999763869735125, -0.20637555723866208, -0.003024742916772738; // right arm
    // clang-format on

    iDynTree::KinDynComputations kinDyn;
    kinDyn.loadRobotModel(ml.model());

    // get the frame associated to l_sole
    auto lSoleFrame = kinDyn.model().getFrameIndex("l_sole");
    if (lSoleFrame == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Unable to find the frame named l_sole.", logPrefix);
        return false;
    }
    auto linkBaseIndex = kinDyn.model().getFrameLink(lSoleFrame);
    auto linkFrameName = kinDyn.model().getLinkName(linkBaseIndex);

    auto l_sole_H_l_foot = kinDyn.getRelativeTransform(lSoleFrame, linkBaseIndex);

    // set kindybase
    kinDyn.setFloatingBase(linkFrameName);

    iDynTree::VectorDynSize dummyVel, jointPositionsiDyn;
    dummyVel.resize(jointPositions.size());
    jointPositionsiDyn.resize(jointPositions.size());
    iDynTree::toEigen(jointPositionsiDyn) = jointPositions;
    dummyVel.zero();

    iDynTree::Vector3 dummyGravity;
    dummyGravity.zero();
    kinDyn.setRobotState(l_sole_H_l_foot,
                         jointPositionsiDyn,
                         iDynTree::Twist::Zero(),
                         dummyVel,
                         dummyGravity);

    manif::SE3d leftFootPose = BipedalLocomotion::Conversions::toManifPose(
        kinDyn.getWorldTransform(ml.model().getFrameIndex("l_sole")));
    manif::SE3d rightFootPose = BipedalLocomotion::Conversions::toManifPose(
        kinDyn.getWorldTransform(ml.model().getFrameIndex("r_sole")));

    manif::SE3d basePose = BipedalLocomotion::Conversions::toManifPose(
        kinDyn.getWorldTransform(ml.model().getFrameIndex("root_link")));

    auto leftFootPoseNewPosition = leftFootPose.translation();
    leftFootPoseNewPosition[0] = -basePose.translation()[0] + leftFootPose.translation()[0];
    leftFootPoseNewPosition[1] = -basePose.translation()[1] + leftFootPose.translation()[1];
    leftFootPoseNewPosition[2] = 0;
    leftFootPose = manif::SE3d(leftFootPoseNewPosition, manif::SO3d::Identity());

    auto rightFootPoseNewPosition = rightFootPose.translation();
    rightFootPoseNewPosition[0] = -basePose.translation()[0] + rightFootPose.translation()[0];
    rightFootPoseNewPosition[1] = -basePose.translation()[1] + rightFootPose.translation()[1];
    rightFootPoseNewPosition[2] = 0;

    iDynTree::Rotation R;
    auto rpyTMP = kinDyn.getWorldTransform("r_sole").getRotation().asRPY();

    rightFootPose = manif::SE3d(rightFootPoseNewPosition,
                                BipedalLocomotion::Conversions::toManifRot(
                                    iDynTree::Rotation::RPY(0, 0, rpyTMP(2))));

    auto newBasePosition = basePose.translation();
    newBasePosition[0] = 0;
    newBasePosition[1] = 0;

    basePose = manif::SE3d(newBasePosition, basePose.quat());

    BipedalLocomotion::Contacts::EstimatedContact leftFoot, rightFoot;

    leftFoot.isActive = true;
    leftFoot.name = "left_foot";
    leftFoot.index = ml.model().getFrameIndex("l_sole");
    leftFoot.switchTime = 0s;
    leftFoot.pose = leftFootPose;

    rightFoot.isActive = true;
    rightFoot.name = "right_foot";
    rightFoot.index = ml.model().getFrameIndex("r_sole");
    rightFoot.switchTime = 0s;
    rightFoot.pose = rightFootPose;

    iDynTree::Transform leftFootTransformIdyn;
    leftFootTransformIdyn.setPosition(iDynTree::Position(leftFoot.pose.translation()));
    leftFootTransformIdyn.setRotation(iDynTree::Rotation(leftFoot.pose.rotation()));
    kinDyn.setRobotState(leftFootTransformIdyn * l_sole_H_l_foot,
                         jointPositionsiDyn,
                         iDynTree::Twist::Zero(),
                         dummyVel,
                         dummyGravity);

    m_generator.setInitialState(jointPositions, basePose);

    m_joypadPort.open("/centroidal-mpc/joystick:i");

    m_directionalInput.motionDirection.setZero();
    m_directionalInput.facingDirection.setZero();

    m_directionalInput.facingDirection << 0, 0;
    m_directionalInput.motionDirection << 0, 0;

    BipedalLocomotion::log()->info("{} Right foot pose {}.",
                                   logPrefix,
                                   rightFoot.pose.coeffs().transpose());
    BipedalLocomotion::log()->info("{} Left foot pose {}.",
                                   logPrefix,
                                   leftFoot.pose.coeffs().transpose());
    BipedalLocomotion::log()->info("{} Initialization completed.", logPrefix);

    return true;
}

const CentroidalMPCBlock::Output& CentroidalMPCBlock::getOutput() const
{
    return m_output;
}

bool CentroidalMPCBlock::setInput(const Input& input)
{
    m_inputValid = input.isValid;

    if (!input.isValid)
    {
        return true;
    }

    m_input = input;
    return true;
}

bool CentroidalMPCBlock::advance()
{
    namespace blf = ::BipedalLocomotion;
    using BipedalLocomotion::log;

    for (int i = 0; i < m_comFrequencyAdapter.outputTimeKnots.size(); i++)
    {
        m_comFrequencyAdapter.outputTimeKnotsAbsolute[i]
            = m_absoluteTime + m_comFrequencyAdapter.outputTimeKnots[i];
    }
    for (int i = 0; i < m_angularMomentumFrequencyAdapter.outputTimeKnots.size(); i++)
    {
        m_angularMomentumFrequencyAdapter.outputTimeKnotsAbsolute[i]
            = m_absoluteTime + m_angularMomentumFrequencyAdapter.outputTimeKnots[i];
    }

    auto tic = std::chrono::steady_clock::now();

    m_output.isValid = false;

    constexpr auto logPrefix = "[CentroidalMPCBlock::advance]";

    if (!m_inputValid)
    {
        log()->warn("{} Input is not valid. Skipping.", logPrefix);
    }

    // MANN trajectory generator does not require any input. Indeed we need the output of the
    // trajectory generator to compute the first step of the whole body controller.
    yarp::sig::Vector* tmp = m_joypadPort.read(false);
    if (tmp != nullptr && tmp->size() == 4)
    {
        m_directionalInput.motionDirection << tmp->operator()(0), tmp->operator()(1);
        m_directionalInput.facingDirection << tmp->operator()(2), tmp->operator()(3);
    }

    using namespace std::chrono_literals;
    if (m_absoluteTime > 2s)
    {
        m_directionalInput.motionDirection << 1, 0;
        m_directionalInput.facingDirection << 1, 0;
    }

    if (m_absoluteTime > 12s)
    {
        m_directionalInput.motionDirection << 1, 0;
        m_directionalInput.facingDirection << 0, 1;
    }


    if (m_absoluteTime > 22s)
    {
         m_directionalInput.motionDirection << 1, 0;
         m_directionalInput.facingDirection << 1, 0;
    }

    if (m_absoluteTime > 32s)
    {
        m_directionalInput.motionDirection << 0, 0;
        m_directionalInput.facingDirection << 1, 0;
    }


    m_output.facingDirection = m_directionalInput.facingDirection;
    m_output.motionDirection = m_directionalInput.motionDirection;

    if (!m_generatorInputBuilder.setInput(m_directionalInput))
    {
        log()->error("{} Unable to set the input to MANN generator.", logPrefix);
        return false;
    }
    if (!m_generatorInputBuilder.advance())
    {
        log()->error("{} Unable to compute one step of the MANN generator builder.", logPrefix);
        return false;
    }

    BipedalLocomotion::ML::MANNTrajectoryGeneratorInput generatorInput;
    generatorInput.mergePointIndex = 0;
    if (m_inputValid && !m_isFirstRun)
    {
        generatorInput.mergePointIndex = m_mergePointIndex;
    }
    generatorInput.desiredFutureBaseTrajectory
        = m_generatorInputBuilder.getOutput().desiredFutureBaseTrajectory;
    generatorInput.desiredFutureBaseVelocities
        = m_generatorInputBuilder.getOutput().desiredFutureBaseVelocities;
    generatorInput.desiredFutureFacingDirections
        = m_generatorInputBuilder.getOutput().desiredFutureFacingDirections;

    if (m_isFirstRun)
    {
        for (int i = 0; i < m_comFrequencyAdapter.inputTimeKnots.size(); i++)
        {
            m_comFrequencyAdapter.inputTimeKnotsAbsolute[i]
                = m_absoluteTime + m_comFrequencyAdapter.inputTimeKnots[i];
        }
        for (int i = 0; i < m_angularMomentumFrequencyAdapter.inputTimeKnots.size(); i++)
        {
            m_angularMomentumFrequencyAdapter.inputTimeKnotsAbsolute[i]
                = m_absoluteTime + m_angularMomentumFrequencyAdapter.inputTimeKnots[i];
        }

        if (!m_generator.setInput(generatorInput))
        {
            log()->error("{} Unable to set the input to MANN generator.", logPrefix);
            return false;
        }
        if (!m_generator.advance())
        {
            log()->error("{} Unable to compute one step of the MANN generator.", logPrefix);
            return false;
        }

        if (!m_generator.isOutputValid())
        {
            log()->error("{} The output of the MANN generator is not valid.", logPrefix);
            return false;
        }

        m_MANNGeneratorOutput = m_generator.getOutput();
    }

    using namespace std::chrono_literals;
    if (!m_isFirstRun && m_absoluteTime % m_mannCallingTime == 0s)
    {
        for (int i = 0; i < m_comFrequencyAdapter.inputTimeKnots.size(); i++)
        {
            m_comFrequencyAdapter.inputTimeKnotsAbsolute[i]
                = m_absoluteTime + m_comFrequencyAdapter.inputTimeKnots[i];
        }
        for (int i = 0; i < m_angularMomentumFrequencyAdapter.inputTimeKnots.size(); i++)
        {
            m_angularMomentumFrequencyAdapter.inputTimeKnotsAbsolute[i]
                = m_absoluteTime + m_angularMomentumFrequencyAdapter.inputTimeKnots[i];
        }

        if (m_mannFuture.valid())
        {
            if (!m_mannFuture.get())
            {
                log()->error("{} Unable to compute one step of the MANN generator.", logPrefix);
                return false;
            }
        } else
        {
            log()->error("{} The future is not valid at t == {}.", logPrefix, m_absoluteTime);
            return false;
        }

        m_MANNGeneratorOutput = m_generator.getOutput();
    }

    if ((m_absoluteTime + m_dT) % m_mannCallingTime == 0s)
    {
        if (!m_generator.setInput(generatorInput))
        {
            log()->error("{} Unable to set the input to MANN generator.", logPrefix);
            return false;
        }

        m_mannFuture
            = std::async(std::launch::async, &CentroidalMPCBlock::runMANNAsynchronously, this);
    }

    if (m_isFirstRun)
    {
        m_output.contactPhaseList = m_MANNGeneratorOutput.phaseList;
        comz0mann = m_MANNGeneratorOutput.comTrajectory.front()[2];
    }

    m_output.regularizedJoints = m_MANNGeneratorOutput.jointPositions.front();
    // for the next steps we need a valid input
    if (!m_inputValid)
    {
        return true;
    }

    // the feedback has been already set in setInput
    auto scaledAngularMomentum = m_MANNGeneratorOutput.angularMomentumTrajectory;
    for (auto& t : scaledAngularMomentum)
    {
        t = t / m_robotMass;
    }

    auto reducedHeightCoM = m_MANNGeneratorOutput.comTrajectory;
    for (auto& t : reducedHeightCoM)
    {
        t[2] = 0.7; //+ (t[2] - comz0mann) / 2;
    }

    m_output.comMANN = reducedHeightCoM.front();
    m_output.angularMomentumMann = scaledAngularMomentum.front();

    auto toc = std::chrono::steady_clock::now();
    m_output.adherentComputationTime = toc - tic;
    tic = toc;

    if (!m_comFrequencyAdapter.spline.setKnots(reducedHeightCoM,
                                               m_comFrequencyAdapter.inputTimeKnotsAbsolute))
    {
        log()->error("{} Unable to set the knots of the com spline.", logPrefix);
        return false;
    }

    if (!m_angularMomentumFrequencyAdapter.spline
             .setKnots(scaledAngularMomentum,
                       m_angularMomentumFrequencyAdapter.inputTimeKnotsAbsolute))
    {
        log()->error("{} Unable to set the knots of the angular momentum spline.", logPrefix);
        return false;
    }

    if (!m_comFrequencyAdapter.spline
             .evaluateOrderedPoints(m_comFrequencyAdapter.outputTimeKnotsAbsolute,
                                    m_comFrequencyAdapter.outputPoints,
                                    m_comFrequencyAdapter.dummy,
                                    m_comFrequencyAdapter.dummy))
    {
        log()->error("{} Unable to evaluate the com spline.", logPrefix);
        return false;
    }

    if (!m_angularMomentumFrequencyAdapter.spline
             .evaluateOrderedPoints(m_angularMomentumFrequencyAdapter.outputTimeKnotsAbsolute,
                                    m_angularMomentumFrequencyAdapter.outputPoints,
                                    m_angularMomentumFrequencyAdapter.dummy,
                                    m_angularMomentumFrequencyAdapter.dummy))
    {
        log()->error("{} Unable to evaluate the angular momentum spline.", logPrefix);
        return false;
    }

    if (!m_controller.setReferenceTrajectory(m_comFrequencyAdapter.outputPoints,
                                             m_angularMomentumFrequencyAdapter.outputPoints))
    {
        log()->error("{} Unable to set the reference trajectory of the MPC.", logPrefix);
        return false;
    }

    BipedalLocomotion::Contacts::ContactPhaseList mannContactPhaseList
        = m_MANNGeneratorOutput.phaseList;
    BipedalLocomotion::Contacts::ContactPhaseList contactPhaseList;
    if (!mannContactPhaseList.forceSampleTime(m_dT))
    {
        log()->error("{} Unable to force the sample time of the contact phase list.", logPrefix);
        return false;
    }

    if (!m_isFirstRun)
    {
        if (!updateContactPhaseList(m_absoluteTime,
                                    mannContactPhaseList,
                                    m_controller.getOutput().contactPhaseList,
                                    contactPhaseList))
        {
            log()->error("{} Unable to update the contact phase list.", logPrefix);
            return false;
        }
    } else
    {
        contactPhaseList = mannContactPhaseList;
    }

    if (!m_controller.setContactPhaseList(contactPhaseList))
    {
        log()->error("{} Unable to set the contact list in the MPC.", logPrefix);
        return false;
    }

    // Here we assume that the total external wrench and the angular momentum are scaled by the
    // robot mass
    if (!m_controller.setState(m_input.com,
                               m_input.dcom,
                               m_input.angularMomentum,
                               m_input.totalExternalWrench))
    {
        log()->error("{} Unable to set the state of the MPC.", logPrefix);
        return false;
    }

    if (!m_controller.advance())
    {
        log()->error("{} Unable to evaluate the output of the MPC.", logPrefix);
        return false;
    }

    // set the controller output
    m_output.controllerOutput = m_controller.getOutput();
    m_output.isValid = true;
    m_output.currentTime = m_absoluteTime;
    m_output.mpcComputationTime = std::chrono::steady_clock::now() - tic;
    m_output.contactPhaseList = m_controller.getOutput().contactPhaseList;
    m_output.mannContactPhaseList = m_MANNGeneratorOutput.phaseList;

    m_isFirstRun = false;

    m_absoluteTime += m_dT;

    return true;
}

bool CentroidalMPCBlock::isOutputValid() const
{
    // for the time beeing we assume that the output is always valid
    return true;
}

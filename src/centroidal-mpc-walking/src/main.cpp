/**
 * @file Main.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <CentroidalMPCWalking/CentroidalMPCBlock.h>
#include <CentroidalMPCWalking/WholeBodyQPBlock.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/AdvanceableRunner.h>
#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/QuitHandler.h>
#include <BipedalLocomotion/System/SharedResource.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <cstdlib>
#include <memory>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

int main(int argc, char* argv[])
{
    constexpr auto errorPrefix = "[main]";

    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // initialize yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        BipedalLocomotion::log()->error("[main] Unable to find YARP network.");
        return EXIT_FAILURE;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.setDefaultConfigFile("dnn-mpc-walking.ini");
    rf.configure(argc, argv);
    auto handler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    handler->set(rf);

    // create the module
    auto wholeBodyQPBlock = std::make_unique<CentroidalMPCWalking::WholeBodyQPBlock>();
    if (!wholeBodyQPBlock->initialize(handler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the whole body block.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    auto input0 = BipedalLocomotion::System::SharedResource<
        CentroidalMPCWalking::WholeBodyQPBlock::Input>::create();
    auto output0 = BipedalLocomotion::System::SharedResource<
        CentroidalMPCWalking::WholeBodyQPBlock::Output>::create();

    BipedalLocomotion::System::AdvanceableRunner<CentroidalMPCWalking::WholeBodyQPBlock>
        wholeBodyRunner;

    if (!wholeBodyRunner.initialize(handler->getGroup("WHOLE_BODY_RUNNER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the whole body runner.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    wholeBodyRunner.setInputResource(input0);
    wholeBodyRunner.setOutputResource(output0);
    wholeBodyRunner.setAdvanceable(std::move(wholeBodyQPBlock));

    ///////// centroidal
    auto centoidalMPCBlock = std::make_unique<CentroidalMPCWalking::CentroidalMPCBlock>();
    if (!centoidalMPCBlock->initialize(handler->getGroup("TRAJECTORY_ADJUSTMENT")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the centoidal mpc block.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    BipedalLocomotion::System::AdvanceableRunner<CentroidalMPCWalking::CentroidalMPCBlock>
        centroidalMPCRunner;

    if (!centroidalMPCRunner.initialize(handler->getGroup("CENTOIDAL_MPC_RUNNER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the centoidal runner.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    centroidalMPCRunner.setInputResource(output0);
    centroidalMPCRunner.setOutputResource(input0);
    centroidalMPCRunner.setAdvanceable(std::move(centoidalMPCBlock));

    BipedalLocomotion::System::handleQuitSignals([&]() {
        centroidalMPCRunner.stop();
        wholeBodyRunner.stop();
    });

    // Run the threads
    auto barrier = BipedalLocomotion::System::Barrier::create(2);

    std::string outcome;
    while (true)
    {
        BipedalLocomotion::log()->info("[main] Do you want to start the experiment [y|n]?");
        std::cin >> outcome;

        if (outcome == "y")
            break;
        if (outcome == "n")
            return EXIT_SUCCESS;
    }

    auto threadWBC = wholeBodyRunner.run(barrier);
    auto threadMPC = centroidalMPCRunner.run(barrier);

    // check if the blocks are alive
    while (wholeBodyRunner.isRunning() && centroidalMPCRunner.isRunning())
    {
        using namespace std::chrono_literals;
        constexpr auto delay = 250ms;

        // release the CPU
        BipedalLocomotion::clock().yield();
        BipedalLocomotion::clock().sleepFor(delay);
    }

    centroidalMPCRunner.stop();
    wholeBodyRunner.stop();

    if (threadMPC.joinable())
    {
        threadMPC.join();
        threadMPC = std::thread();
    }

    if (threadWBC.joinable())
    {
        threadWBC.join();
        threadWBC = std::thread();
    }

    return EXIT_SUCCESS;
}

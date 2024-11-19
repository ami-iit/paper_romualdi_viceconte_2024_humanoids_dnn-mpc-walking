/**
 * @file main.cpp
 * @author Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <cstdlib>
#include <memory>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/AdvanceableRunner.h>
#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/QuitHandler.h>
#include <BipedalLocomotion/System/SharedResource.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>


#include <FakeJoypad/FakeJoypad.h>
#include <FakeJoypad/JoypadProvider.h>

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

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("dnn-mpc-joypad.ini");
    rf.configure(argc, argv);

    auto handler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    handler->set(rf);

    // create the fake joypad
    auto fakeJoypad = std::make_unique<CentroidalMPCWalking::FakeJoypad>();
    if (!fakeJoypad->initialize(handler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the whole body block.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    // create the joypad provider
    auto joypadProvider = std::make_unique<CentroidalMPCWalking::JoypadProvider>();
    if (!joypadProvider->initialize(handler->getGroup("JOYPAD_PROVIDER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the whole body block.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    auto input0 = BipedalLocomotion::System::SharedResource<
        CentroidalMPCWalking::FakeJoypad::Input>::create();
    auto output0 = BipedalLocomotion::System::SharedResource<
        CentroidalMPCWalking::FakeJoypad::Output>::create();

    BipedalLocomotion::System::AdvanceableRunner<CentroidalMPCWalking::FakeJoypad> fakeJoypadRunner;

    if (!fakeJoypadRunner.initialize(handler->getGroup("FAKE_JOYPAD_RUNNER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the whole body runner.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    fakeJoypadRunner.setInputResource(input0);
    fakeJoypadRunner.setOutputResource(output0);
    fakeJoypadRunner.setAdvanceable(std::move(fakeJoypad));

    // create the advanceable runner for the joypad provider
    BipedalLocomotion::System::AdvanceableRunner<CentroidalMPCWalking::JoypadProvider>
        joypadProviderRunner;

    if (!joypadProviderRunner.initialize(handler->getGroup("JOYPAD_PROVIDER_RUNNER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the whole body runner.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    joypadProviderRunner.setInputResource(output0);
    joypadProviderRunner.setOutputResource(input0);
    joypadProviderRunner.setAdvanceable(std::move(joypadProvider));


    BipedalLocomotion::System::handleQuitSignals([&]() { fakeJoypadRunner.stop(); });

    // Run the threads
    auto  barrier = BipedalLocomotion::System::Barrier::create(2);

    auto threadJoypad = fakeJoypadRunner.run(barrier);
    auto threadJoypadProvider = joypadProviderRunner.run(barrier);

    while (fakeJoypadRunner.isRunning() && joypadProviderRunner.isRunning())
    {
        using namespace std::chrono_literals;
        constexpr auto delay = 250ms;

        // release the CPU
        BipedalLocomotion::clock().yield();
        BipedalLocomotion::clock().sleepFor(delay);
    }

    fakeJoypadRunner.stop();
    joypadProviderRunner.stop();

    if (threadJoypad.joinable())
    {
        threadJoypad.join();
    }

    if (threadJoypadProvider.joinable())
    {
        threadJoypadProvider.join();
    }

    return EXIT_SUCCESS;
}

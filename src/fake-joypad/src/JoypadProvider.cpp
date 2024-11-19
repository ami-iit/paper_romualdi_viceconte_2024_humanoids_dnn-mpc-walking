/**
 * @file JoypadProvider.cpp
 * @author Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <BipedalLocomotion/System/SharedResource.h>
#include <FakeJoypad/JoypadProvider.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace CentroidalMPCWalking;

struct JoypadProvider::Impl
{
    yarp::os::BufferedPort<yarp::sig::Vector> port;
    JoypadSignal joypad;

    ~Impl()
    {
        port.close();
    }
};

JoypadProvider::JoypadProvider()
{
    m_pimpl = std::make_unique<Impl>();
}

JoypadProvider::~JoypadProvider() = default;

bool JoypadProvider::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("[JoypadProvider::initialize] The handler is not valid.");
        return false;
    }

    std::string localPortName;
    if (!ptr->getParameter("local", localPortName))
    {
        BipedalLocomotion::log()->error("[JoypadProvider::initialize] Unable to find the 'local' "
                                        "parameter.");
        return false;
    }

    return m_pimpl->port.open(localPortName);
}

bool JoypadProvider::advance()
{
    constexpr bool writeStrict = false;

    auto& output = m_pimpl->port.prepare();
    output.resize(4);
    output[0] = m_pimpl->joypad.leftAnalogX;
    output[1] = m_pimpl->joypad.leftAnalogY;
    output[2] = m_pimpl->joypad.rightAnalogX;
    output[3] = m_pimpl->joypad.rightAnalogY;
    m_pimpl->port.write(writeStrict);

    return true;
}

bool JoypadProvider::setInput(const JoypadSignal& input)
{
    m_pimpl->joypad = input;
    return true;
}

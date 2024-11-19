/**
 * @file JoypadProvider.h
 * @author Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#ifndef JOYPAD_PROVIDER_H
#define JOYPAD_PROVIDER_H

#include <memory>

#include <BipedalLocomotion/System/Sink.h>
#include <FakeJoypad/FakeJoypad.h>

namespace CentroidalMPCWalking
{

/**
 * @brief JoypadProvider is a class used to provide the joystick signal in the YARP network.
 */
class JoypadProvider : public BipedalLocomotion::System::Sink<JoypadSignal>
{
public:
    /**
     * @brief Constructor.
     */
    JoypadProvider();

    /**
     * @brief Destructor.
     */
    ~JoypadProvider();

    /**
     * @brief Initialize the provider.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are read:
     * - `local`: the nome of the local port. It will be the port used to send the joystick signal.
     * @return true if the initialization is successful, false otherwise.
     */
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                        handler) override;

    /**
     * @brief Send the joypad signal to the YARP network.
     * @return true if successful, false otherwise.
     */
    bool advance() override;

    /**
     * @brief Set the input of the provider.
     * @param input the input of the system.
     * @return true if the input is valid, false otherwise.
     */
    bool setInput(const JoypadSignal& input) override;

private:
    struct Impl; /**< Implementation */
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to the implementation */
};

} // namespace CentroidalMPCWalking

#endif // JOYPAD_PROVIDER_H

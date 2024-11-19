/**
 * @file FakeJoypad.h
 * @author Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#ifndef FAKE_JOYSTICK_H
#define FAKE_JOYSTICK_H

#include <memory>

#include <BipedalLocomotion/System/Source.h>

namespace CentroidalMPCWalking
{

/**
 * @brief JoypadSignal is the output of the FakeJoypad class.
 */
struct JoypadSignal
{
    double leftAnalogX = 0.0; /**< Left analog X. */
    double leftAnalogY = 0.0; /**< Left analog Y. */
    double rightAnalogX = 0.0; /**< Right analog X. */
    double rightAnalogY = 0.0; /**< Right analog Y. */
};

/**
 * @brief FakeJoypad is a class used to simulate the joystick.
 * @note The class is used to simulate the joystick. The user can use the keyboard to simulate the
 * joystick and send the command to the walking application.
 * WASD is used to simulate the left analog stick. The arrow keys are used to simulate the right
 * analog.
 */
class FakeJoypad : public BipedalLocomotion::System::Source<JoypadSignal>
{
public:
    /**
     * @brief Constructor.
     */
    FakeJoypad();

    /**
     * @brief Destructor.
     */
    ~FakeJoypad();

    /**
     * @brief Compute the joystick signal from the keyboard.
     * @return true if the computation is successful, false otherwise.
     */
    bool advance() override;

    /**
     * @brief Get the output of the system.
     * @return the output of the system.
     */
    const JoypadSignal& getOutput() const override;

    /**
     * @brief Check if the output is valid.
     * @return true if the output is valid, false otherwise.
     */
    bool isOutputValid() const override;

private:
    struct Impl; /**< Implementation of the class. */
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to the implementation of the class. */
};

} // namespace CentroidalMPCWalking

#endif // FAKE_JOYSTICK_H

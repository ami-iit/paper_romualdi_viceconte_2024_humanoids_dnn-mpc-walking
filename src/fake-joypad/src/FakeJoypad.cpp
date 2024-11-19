/**
 * @file FakeJoypad.cpp
 * @author Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <FakeJoypad/FakeJoypad.h>
#include <SDL/SDL.h>

using namespace CentroidalMPCWalking;

struct FakeJoypad::Impl
{
    JoypadSignal joypad;

    // number of keys
    static constexpr int NUM_KEYS = SDLK_LAST;

    Uint32 emphasisColor;
    Uint32 color;
    bool keyState[NUM_KEYS] = {false}; // Array to track key states

    Impl()
    {
        // Initialize SDL
        SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);

        // Set the video mode to 640x220 with 32-bit pixels
        SDL_SetVideoMode(640, 220, 32, SDL_SWSURFACE);

        // add title to the window
        SDL_WM_SetCaption("Fake Joypad", NULL);

        // Blue color for normal lines
        this->color = SDL_MapRGB(SDL_GetVideoSurface()->format, 66, 133, 244);

        // Orange color for emphasized lines
        this->emphasisColor = SDL_MapRGB(SDL_GetVideoSurface()->format, 244, 180, 0);

        // Draw the normal left cross
        this->drawCross(SDL_GetVideoSurface(), 160, 120, 50, this->color);

        // Draw the normal right cross
        this->drawCross(SDL_GetVideoSurface(), 480, 120, 50, this->color);

        // Update the screen
        SDL_Flip(SDL_GetVideoSurface());
    }

    ~Impl()
    {
        // Quit SDL
        SDL_Quit();
    }

    void drawCross(SDL_Surface* screen, int x, int y, int size, Uint32 color)
    {
        int thickness = 5; // Thickness of the cross segments

        // Vertical segment
        SDL_Rect verticalSegment = {static_cast<Sint16>(x - thickness / 2),
                                    static_cast<Sint16>(y - size / 2),
                                    static_cast<Uint16>(thickness),
                                    static_cast<Uint16>(size)};
        SDL_FillRect(screen, &verticalSegment, color);

        // Horizontal segment
        SDL_Rect horizontalSegment = {static_cast<Sint16>(x - size / 2),
                                      static_cast<Sint16>(y - thickness / 2),
                                      static_cast<Uint16>(size),
                                      static_cast<Uint16>(thickness)};
        SDL_FillRect(screen, &horizontalSegment, color);
    }

    void emphasizeLine(SDL_Surface* screen,
                       int x,
                       int y,
                       int size,
                       bool up,
                       bool down,
                       bool left,
                       bool right,
                       Uint32 emphasisColor)
    {
        int thickness = 5; // Thickness of the emphasized line

        // Emphasize vertical line if 'up' is true
        if (up)
        {
            SDL_Rect upSegment = {static_cast<Sint16>(x - thickness / 2),
                                  static_cast<Sint16>(y - size / 2),
                                  static_cast<Uint16>(thickness),
                                  static_cast<Uint16>(size / 2)};
            SDL_FillRect(screen, &upSegment, emphasisColor);
        }

        // Emphasize horizontal line if 'left' is true
        if (left)
        {
            SDL_Rect leftSegment = {static_cast<Sint16>(x - size / 2),
                                    static_cast<Sint16>(y - thickness / 2),
                                    static_cast<Uint16>(size / 2),
                                    static_cast<Uint16>(thickness)};
            SDL_FillRect(screen, &leftSegment, emphasisColor);
        }

        // Emphasize vertical line if 'down' is true
        if (down)
        {
            SDL_Rect downSegment = {static_cast<Sint16>(x - thickness / 2),
                                    static_cast<Sint16>(y + thickness / 2),
                                    static_cast<Uint16>(thickness),
                                    static_cast<Uint16>(size / 2)};
            SDL_FillRect(screen, &downSegment, emphasisColor);
        }

        // Emphasize horizontal line if 'right' is true
        if (right)
        {
            SDL_Rect rightSegment = {static_cast<Sint16>(x + thickness / 2),
                                     static_cast<Sint16>(y - thickness / 2),
                                     static_cast<Uint16>(size / 2),
                                     static_cast<Uint16>(thickness)};
            SDL_FillRect(screen, &rightSegment, emphasisColor);
        }
    }
};

FakeJoypad::FakeJoypad()
{
    m_pimpl = std::make_unique<Impl>();
}

FakeJoypad::~FakeJoypad() = default;

bool FakeJoypad::advance()
{
    SDL_Event event;
    if (SDL_PollEvent(&event))
    {
        if (event.type == SDL_QUIT)
        {
            return false;
        }

        // Clear the screen
        SDL_FillRect(SDL_GetVideoSurface(),
                     NULL,
                     SDL_MapRGB(SDL_GetVideoSurface()->format, 0, 0, 0));

        // Draw the normal left cross
        m_pimpl->drawCross(SDL_GetVideoSurface(), 160, 120, 50, m_pimpl->color);

        // Draw the normal right cross
        m_pimpl->drawCross(SDL_GetVideoSurface(), 480, 120, 50, m_pimpl->color);

        if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP)
        {
            // Update the key state array based on key events
            m_pimpl->keyState[event.key.keysym.sym] = (event.type == SDL_KEYDOWN);

            // Emphasize the lines based on key states
            m_pimpl->emphasizeLine(SDL_GetVideoSurface(),
                                   160,
                                   120,
                                   50,
                                   m_pimpl->keyState[SDLK_w],
                                   m_pimpl->keyState[SDLK_s],
                                   m_pimpl->keyState[SDLK_a],
                                   m_pimpl->keyState[SDLK_d],
                                   m_pimpl->emphasisColor);

            // Emphasize the lines based on key states
            m_pimpl->emphasizeLine(SDL_GetVideoSurface(),
                                   480,
                                   120,
                                   50,
                                   m_pimpl->keyState[SDLK_UP],
                                   m_pimpl->keyState[SDLK_DOWN],
                                   m_pimpl->keyState[SDLK_LEFT],
                                   m_pimpl->keyState[SDLK_RIGHT],
                                   m_pimpl->emphasisColor);

            // Update the joystick structure based on key states
            m_pimpl->joypad.leftAnalogX = m_pimpl->keyState[SDLK_w] - m_pimpl->keyState[SDLK_s];
            m_pimpl->joypad.leftAnalogY = m_pimpl->keyState[SDLK_a] - m_pimpl->keyState[SDLK_d];
            m_pimpl->joypad.rightAnalogX
                = m_pimpl->keyState[SDLK_UP] - m_pimpl->keyState[SDLK_DOWN];
            m_pimpl->joypad.rightAnalogY
                = m_pimpl->keyState[SDLK_LEFT] - m_pimpl->keyState[SDLK_RIGHT];

            // Update the screen
            SDL_Flip(SDL_GetVideoSurface());
        }
    }

    return true;
}

const JoypadSignal& FakeJoypad::getOutput() const
{
    return m_pimpl->joypad;
}

bool FakeJoypad::isOutputValid() const
{
    // The output is always valid
    return true;
}

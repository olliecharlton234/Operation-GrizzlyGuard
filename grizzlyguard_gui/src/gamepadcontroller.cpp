// ============================================================
// GAMEPADCONTROLLER.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "gamepadcontroller.h"

#include <SDL.h>
#include <cmath>
#include <algorithm>

namespace {
constexpr double kDeadzone = 0.15; // ignore tiny stick drift near center
constexpr double kPollIntervalMs = 50.0; // 20Hz - plenty for manual flight
constexpr double kChangeThreshold = 0.02; // only re-emit if value moved this much

// ============================================================
// APPLY DEADZONE
// ============================================================
double applyDeadzone(double value)
{
    if (std::abs(value) < kDeadzone)
        return 0.0;
    // Rescale so output still reaches -1..1 just past the deadzone,
    // instead of jumping straight from 0 to 0.15.
    double sign = value > 0 ? 1.0 : -1.0;
    return sign * (std::abs(value) - kDeadzone) / (1.0 - kDeadzone);
}

// ============================================================
// NORMALIZE AXIS
// ============================================================
double normalizeAxis(Sint16 raw)
{
    return applyDeadzone(static_cast<double>(raw) / 32768.0);
}
}

// ============================================================
// GAMEPAD CONTROLLER
// ============================================================
GamepadController::GamepadController(QObject *parent)
    : QObject(parent)
{
    // SDL_INIT_GAMECONTROLLER also brings in joystick support.
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        qWarning("SDL_Init failed: %s", SDL_GetError());
    }

    connect(&m_pollTimer, &QTimer::timeout, this, &GamepadController::pollAxes);
    connect(&m_connectionCheckTimer, &QTimer::timeout, this, &GamepadController::pollForConnection);
    m_connectionCheckTimer.start(2000); // check every 2s if nothing connected yet
}

// ============================================================
// ~GAMEPAD CONTROLLER
// ============================================================
GamepadController::~GamepadController()
{
    if (m_controller)
        SDL_GameControllerClose(static_cast<SDL_GameController *>(m_controller));
    SDL_Quit();
}

// ============================================================
// OPEN FIRST AVAILABLE
// ============================================================
bool GamepadController::openFirstAvailable()
{
    SDL_GameControllerUpdate();

    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            m_controller = SDL_GameControllerOpen(i);
            if (m_controller) {
                m_pollTimer.start(static_cast<int>(kPollIntervalMs));
                emit connectedChanged(true);
                return true;
            }
        }
    }
    return false;
}

// ============================================================
// POLL FOR CONNECTION
// ============================================================
void GamepadController::pollForConnection()
{
    if (isConnected())
        return; // already have one, nothing to do

    if (openFirstAvailable())
        m_connectionCheckTimer.stop(); // found one, no need to keep polling
}

// ============================================================
// POLL AXES
// ============================================================
void GamepadController::pollAxes()
{
    if (!m_controller)
        return;

    auto *controller = static_cast<SDL_GameController *>(m_controller);
    SDL_GameControllerUpdate();

    // Detect disconnect (e.g. USB unplugged mid-flight).
    if (!SDL_GameControllerGetAttached(controller)) {
        SDL_GameControllerClose(controller);
        m_controller = nullptr;
        m_pollTimer.stop();
        emit connectedChanged(false);
        emit moveRequested(0.0, 0.0, 0.0, 0.0); // fail safe: stop movement
        m_connectionCheckTimer.start(2000);
        return;
    }

    // Standard layout: left stick = forward/strafe, right stick Y = altitude,
    // right stick X = yaw. Flip signs here if your controller feels inverted.
    double forward = -normalizeAxis(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY));
    double right   =  normalizeAxis(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX));
    double up      = -normalizeAxis(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY));
    double yaw     =  normalizeAxis(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTX));

    bool changed = std::abs(forward - m_lastForward) > kChangeThreshold
                   || std::abs(right - m_lastRight) > kChangeThreshold
                   || std::abs(up - m_lastUp) > kChangeThreshold
                   || std::abs(yaw - m_lastYaw) > kChangeThreshold;

    if (changed) {
        m_lastForward = forward;
        m_lastRight = right;
        m_lastUp = up;
        m_lastYaw = yaw;
        emit moveRequested(forward, right, up, yaw);
    }

    // Face buttons for takeoff/land - A = takeoff, B = land (adjust to taste).
    static bool wasA = false, wasB = false;
    bool isA = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A);
    bool isB = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_B);
    if (isA && !wasA) emit takeoffRequested();
    if (isB && !wasB) emit landRequested();
    wasA = isA;
    wasB = isB;
}

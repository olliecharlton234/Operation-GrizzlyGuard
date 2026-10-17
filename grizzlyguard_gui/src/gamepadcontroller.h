#ifndef GAMEPADCONTROLLER_H
#define GAMEPADCONTROLLER_H


#include <QObject>
#include <QTimer>

// Wraps SDL2's game controller API so the rest of the app can treat a
// physical gamepad exactly like the keyboard: it just emits the same
// moveRequested-style signal. Requires SDL2 (see CMakeLists.txt).
//
// Deadzone and axis mapping are set up for a standard Xbox/PlayStation
// style layout: left stick = forward/strafe, right stick = altitude/yaw.
// Adjust axis indices in the .cpp if your controller maps differently.
class GamepadController : public QObject
{
    Q_OBJECT

public:
    explicit GamepadController(QObject *parent = nullptr);
    ~GamepadController() override;

    // Call once at startup. Returns true if a controller was found and
    // opened. Safe to call even if no controller is plugged in yet - use
    // pollForConnection() to keep checking.
    bool openFirstAvailable();

    bool isConnected() const { return m_controller != nullptr; }

public slots:
    // Keep calling this periodically (e.g. every couple of seconds) if no
    // controller was connected at startup, so plugging one in later works.
    void pollForConnection();

signals:
    // Same shape as ManualControlWidget::moveRequested - connect both to
    // the same drone-command slot.
    void moveRequested(double forward, double right, double up, double yawRate);
    void takeoffRequested();
    void landRequested();
    void connectedChanged(bool connected);

private slots:
    void pollAxes(); // reads current stick state and emits moveRequested

private:
    void *m_controller = nullptr; // SDL_GameController*, void* to avoid
    // leaking SDL headers into this .h
    QTimer m_pollTimer;
    QTimer m_connectionCheckTimer;

    // Remember last emitted values so we don't spam signals when nothing
    // has changed (helps avoid flooding your drone command link).
    double m_lastForward = 0.0, m_lastRight = 0.0, m_lastUp = 0.0, m_lastYaw = 0.0;
};


#endif // GAMEPADCONTROLLER_H

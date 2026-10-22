// ============================================================
// MAINWINDOW.H
// Public interface and member declarations
// ============================================================

#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <QProcess>

class QStackedWidget;
class QPushButton;
class CameraViewWidget;
class MapWidget;
class TelemetryPanel;
class ManualControlWidget;
class GamepadController;
class PerimeterAlertDialog;
class DroneBackend;

/**
 * @brief Main window for monitoring and manual control.
 *
 * Owns the overview, manual control page and alert presentation. ROS 2 runs
 * in DroneBackend. The perimeter status and flight actions still require
 * agreed interfaces from the team's simulation.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit MainWindow(QWidget *parent = nullptr);

    // -------------------------------------------------------
    // PRIVATE SLOTS
    // -------------------------------------------------------
private slots:
    void onEmergencyStop();
    void onReturnToHome();
    void onPageChanged(int index);
    void onManualModeButtonClicked(bool checked); // two-step: asks for confirmation first

    // Connected to DroneBackend's incoming signals.
    void onDronePositionChanged(double x, double y);
    void onBearDetected(const QString &bearId, double x, double y);

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    QWidget *buildOverviewTab();
    QProcess *bridgeProcess = nullptr;

    QStackedWidget *m_pages; // Overview page and Manual Control page (no tab bar)
    CameraViewWidget *m_cameraView;
    MapWidget *m_mapView;
    TelemetryPanel *m_telemetryPanel;
    ManualControlWidget *m_manualControl;
    GamepadController *m_gamepad;
    DroneBackend *m_backend;

    PerimeterAlertDialog *m_alertDialog;

    QPushButton *m_emergencyStopButton;
    QPushButton *m_rthButton;
    QPushButton *m_manualModeButton = nullptr; // switches to/from the Manual Control tab
};




#endif // MAINWINDOW_H

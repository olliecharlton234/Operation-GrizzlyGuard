#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>

class QTabWidget;
class QPushButton;
class CameraViewWidget;
class MapWidget;
class TelemetryPanel;
class ManualControlWidget;
class GamepadController;
class DroneBackend;

// Two tabs: "Overview" (camera, map, telemetry - always the default) and
// "Manual Control" (the ONLY other tab; has its own camera + map, kept in
// sync with the Overview tab's). Return to Home and Emergency Stop live
// outside the tab widget so they're reachable no matter which tab is active.
//
// ALL communication with the drone/simulation goes through DroneBackend
// (see dronebackend.h) - MainWindow never talks to real hardware or
// simulation code directly. When the real backend is ready, only
// dronebackend.cpp needs to change.
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void onEmergencyStop();
    void onReturnToHome();
    void onTabChanged(int index);

    // Connected to DroneBackend's incoming signals.
    void onDronePositionChanged(double latitude, double longitude);
    void onBearDetected(const QString &bearId, double latitude, double longitude);
    void onBatteryChanged(int percent);

private:
    QWidget *buildOverviewTab();
    void recomputePerimeterStatus();

    QTabWidget *m_tabs;
    CameraViewWidget *m_cameraView;
    MapWidget *m_mapView;
    TelemetryPanel *m_telemetryPanel;
    ManualControlWidget *m_manualControl;
    GamepadController *m_gamepad;
    DroneBackend *m_backend;

    QPushButton *m_emergencyStopButton;
    QPushButton *m_rthButton;
};



#endif // MAINWINDOW_H

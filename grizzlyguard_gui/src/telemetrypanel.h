// ============================================================
// TELEMETRYPANEL.H
// Public interface and member declarations
// ============================================================

#ifndef TELEMETRYPANEL_H
#define TELEMETRYPANEL_H

#include <QWidget>

class QLabel;
class QProgressBar;
class QTimer;
class TrafficLightWidget;
class DeterrentIndicatorWidget;

/**
 * @brief Drone telemetry and status display.
 *
 * Shows drone name, a locally simulated battery (drains 1% every 18 s),
 * the perimeter traffic light and the deterrent indicator.
 */
class TelemetryPanel : public QWidget
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit TelemetryPanel(QWidget *parent = nullptr);

    // -------------------------------------------------------
    // PUBLIC SLOTS
    // -------------------------------------------------------
public slots:
    void setDroneName(const QString &name);
    void setPerimeterDistance(double distanceMeters);
    void setPerimeterStatus(int status);   // 1 = SAFE, 2 = CAUTION, 3 = DANGER, other = UNKNOWN
    void setDeterrentDeployed(bool deployed);
    void setDroneStatus(const QString &status);

    // -------------------------------------------------------
    // PRIVATE SLOTS
    // -------------------------------------------------------
private slots:
    void updateBatteryLevel();             // called by m_batteryTimer

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    QLabel *m_nameLabel = nullptr;

    QLabel *m_statusLabel = nullptr;

    QProgressBar *m_batteryBar = nullptr;
    QLabel *m_batteryLabel = nullptr;
    QTimer *m_batteryTimer = nullptr;
    int m_batteryLevel = 100;              // starts full

    TrafficLightWidget *m_trafficLight = nullptr;
    DeterrentIndicatorWidget *m_deterrentIndicator = nullptr;
    QLabel *m_deterrentLabel = nullptr;
};

#endif // TELEMETRYPANEL_H
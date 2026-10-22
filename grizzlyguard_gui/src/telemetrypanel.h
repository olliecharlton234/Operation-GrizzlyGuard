// ============================================================
// TELEMETRYPANEL.H
// Telemetry panel interface and member declarations
// ============================================================

#ifndef TELEMETRYPANEL_H
#define TELEMETRYPANEL_H

#include <QWidget>
#include <QString>

class QLabel;
class QProgressBar;
class TrafficLightWidget;
class DeterrentIndicatorWidget;


// ============================================================
// TELEMETRY PANEL CLASS
// ============================================================

class TelemetryPanel : public QWidget
{
    Q_OBJECT

public:
    explicit TelemetryPanel(QWidget *parent = nullptr);


    // ========================================================
    // PUBLIC SLOTS
    // ========================================================

public slots:

    // Update the drone's displayed name
    void setDroneName(const QString &name);

    // Update the simulated battery percentage
    void setBatteryPercent(int percent);

    // Update the perimeter warning indicator
    void setPerimeterDistance(double distanceMeters);

    // Update the deterrent status
    void setDeterrentDeployed(bool deployed);


    // ========================================================
    // PRIVATE MEMBERS
    // ========================================================

private:

    // Drone identification
    QLabel *m_nameLabel = nullptr;

    // Simulated battery
    QLabel *m_batteryLabel = nullptr;
    QProgressBar *m_batteryBar = nullptr;

    // Perimeter status
    TrafficLightWidget *m_trafficLight = nullptr;

    // Deterrent status
    DeterrentIndicatorWidget *m_deterrentIndicator = nullptr;
    QLabel *m_deterrentLabel = nullptr;
};

#endif // TELEMETRYPANEL_H

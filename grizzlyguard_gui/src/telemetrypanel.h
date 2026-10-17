#ifndef TELEMETRYPANEL_H
#define TELEMETRYPANEL_H

#include <QWidget>

class QLabel;
class QProgressBar;
class TrafficLightWidget;
class DeterrentIndicatorWidget;

// Shows drone name, battery level, the perimeter traffic light, and the
// deterrent-deployed indicator together in one sidebar panel.
class TelemetryPanel : public QWidget
{
    Q_OBJECT

public:
    explicit TelemetryPanel(QWidget *parent = nullptr);

public slots:
    void setDroneName(const QString &name);
    void setBatteryPercent(int percent); // 0-100
    void setPerimeterDistance(double distanceMeters);
    void setDeterrentDeployed(bool deployed);

private:
    QLabel *m_nameLabel;
    QProgressBar *m_batteryBar;
    QLabel *m_batteryLabel;
    TrafficLightWidget *m_trafficLight;
    DeterrentIndicatorWidget *m_deterrentIndicator;
    QLabel *m_deterrentLabel;
};

#endif // TELEMETRYPANEL_H

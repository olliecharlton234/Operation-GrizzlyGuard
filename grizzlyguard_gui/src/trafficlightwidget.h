#ifndef TRAFFICLIGHTWIDGET_H
#define TRAFFICLIGHTWIDGET_H


#include <QWidget>

// Simple 3-light indicator showing how close a bear is to the perimeter.
class TrafficLightWidget : public QWidget
{
    Q_OBJECT

public:
    enum class Level { Safe, Caution, Danger };

    explicit TrafficLightWidget(QWidget *parent = nullptr);

    // Call this whenever you recompute bear-to-perimeter distance.
    void setLevel(Level level);

    // Convenience: feed a raw distance (meters) + thresholds and let the
    // widget decide the light. Distances below dangerMeters = red,
    // below cautionMeters = yellow, otherwise green.
    void setDistanceToPerimeter(double distanceMeters,
                                double cautionMeters = 200.0,
                                double dangerMeters = 75.0);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    Level m_level = Level::Safe;
};

#endif // TRAFFICLIGHTWIDGET_H

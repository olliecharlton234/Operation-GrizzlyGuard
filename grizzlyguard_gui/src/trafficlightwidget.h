// ============================================================
// TRAFFICLIGHTWIDGET.H
// Public interface and member declarations
// ============================================================

#ifndef TRAFFICLIGHTWIDGET_H
#define TRAFFICLIGHTWIDGET_H


#include <QWidget>

// Three-state warning indicator; all lamps stay dim until the perimeter system reports a level.
class TrafficLightWidget : public QWidget
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    enum class Level { Unknown, Safe, Caution, Danger };

    explicit TrafficLightWidget(QWidget *parent = nullptr);

    // Call this whenever you recompute bear-to-perimeter distance.
    void setLevel(Level level);

    // Convenience: feed a raw distance (meters) + thresholds and let the
    // widget decide the light. Distances below dangerMeters = red,
    // below cautionMeters = yellow, otherwise green.
    void setDistanceToPerimeter(double distanceMeters,
                                double cautionMeters = 200.0,
                                double dangerMeters = 75.0);

    // -------------------------------------------------------
    // PROTECTED
    // -------------------------------------------------------
protected:
    void paintEvent(QPaintEvent *event) override;

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    Level m_level = Level::Unknown;
};

#endif // TRAFFICLIGHTWIDGET_H

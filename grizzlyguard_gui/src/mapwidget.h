// ============================================================
// MAPWIDGET.H
// Public interface and member declarations
// ============================================================

#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsPixmapItem>
#include <QMap>
#include <QPointF>
#include <QString>

class QPushButton;
class QResizeEvent;

/**
 * @brief Displays a fixed overhead Gazebo world image with live X/Y markers.
 *
 * The image is not a live camera feed. Positions must be supplied in the
 * Gazebo world frame, or transformed to that frame by the ROS integration.
 * World bounds are image calibration values, not the terrain's dimensions.
 * Perimeter logic is deliberately not implemented here.
 */
class MapWidget : public QGraphicsView
{
    Q_OBJECT
    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit MapWidget(QWidget *parent = nullptr);

    void setWorldBounds(double minX, double maxX, double minY, double maxY);
    void setDronePosition(double x, double y, double headingDegrees = 0.0);
    void upsertBear(const QString &bearId, double x, double y);
    void removeBear(const QString &bearId);
    void openExpandedMap();

    // -------------------------------------------------------
    // PROTECTED
    // -------------------------------------------------------
protected:
    void resizeEvent(QResizeEvent *event) override;

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    QPointF projectToScene(double x, double y) const;
    void updateMarkers();
    void fitMapToPanel();

    QGraphicsScene m_scene;
    QGraphicsPixmapItem *m_background = nullptr;
    QGraphicsEllipseItem *m_droneItem = nullptr;
    QMap<QString, QGraphicsEllipseItem *> m_bearItems;
    QMap<QString, QPointF> m_bearPositions;
    QPointF m_dronePosition;
    bool m_hasDronePosition = false;
    QPushButton *m_expandButton = nullptr;

    // Approximate values. Calibrate against known locations in world_map.png.
    double m_minX = -195.0;
    double m_maxX = 209.0;
    double m_minY = -126.0;
    double m_maxY = 269.0;
};

#endif // MAPWIDGET_H

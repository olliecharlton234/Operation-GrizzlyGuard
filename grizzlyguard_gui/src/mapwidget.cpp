#include "mapwidget.h"

#include <QtMath>
#include <QGraphicsTextItem>
#include <limits>

static constexpr double kEarthRadiusMeters = 6371000.0;

MapWidget::MapWidget(QWidget *parent)
    : QGraphicsView(parent)
{
    setScene(&m_scene);
    setRenderHint(QPainter::Antialiasing);
    setBackgroundBrush(QColor(20, 60, 30)); // "forest" green backdrop
    setDragMode(QGraphicsView::ScrollHandDrag);

    m_perimeterItem = m_scene.addEllipse(QRectF(), QPen(QColor(255, 200, 0), 2, Qt::DashLine));

    // Drone marker: solid blue dot with a white outline.
    m_droneItem = m_scene.addEllipse(QRectF(-6, -6, 12, 12),
                                     QPen(Qt::white), QBrush(QColor(30, 100, 255)));
    m_droneItem->setZValue(10);
}

void MapWidget::setHomePosition(double latitude, double longitude)
{
    m_homeLat = latitude;
    m_homeLon = longitude;

    // Re-project everything relative to the new home point.
    m_perimeterItem->setRect(-m_perimeterMeters, -m_perimeterMeters,
                             m_perimeterMeters * 2, m_perimeterMeters * 2);

    for (auto it = m_bearLatLon.constBegin(); it != m_bearLatLon.constEnd(); ++it)
        upsertBear(it.key(), it.value().x(), it.value().y());

    fitInView(m_scene.itemsBoundingRect().adjusted(-50, -50, 50, 50), Qt::KeepAspectRatio);
}

void MapWidget::setPerimeterRadius(double meters)
{
    m_perimeterMeters = meters;
    m_perimeterItem->setRect(-meters, -meters, meters * 2, meters * 2);
}

// Equirectangular approximation - fine for perimeter-scale distances
// (hundreds of meters to a few km). Swap for a proper geodesic calc if the
// drone roams farther.
QPointF MapWidget::projectToScene(double latitude, double longitude) const
{
    double dLat = qDegreesToRadians(latitude - m_homeLat);
    double dLon = qDegreesToRadians(longitude - m_homeLon);
    double meanLat = qDegreesToRadians((latitude + m_homeLat) / 2.0);

    double x = dLon * kEarthRadiusMeters * qCos(meanLat);
    double y = -dLat * kEarthRadiusMeters; // north = up (negative y in Qt scene)

    return QPointF(x, y) / m_metersPerSceneUnit;
}

void MapWidget::setDronePosition(double latitude, double longitude, double headingDegrees)
{
    QPointF pos = projectToScene(latitude, longitude);
    m_droneItem->setPos(pos);
    Q_UNUSED(headingDegrees); // rotate a triangle/icon here if you want heading shown
}

void MapWidget::upsertBear(const QString &bearId, double latitude, double longitude)
{
    m_bearLatLon[bearId] = QPointF(latitude, longitude);
    QPointF pos = projectToScene(latitude, longitude);

    auto it = m_bearItems.find(bearId);
    if (it == m_bearItems.end()) {
        // Bear marker: solid red dot with a black outline.
        auto *item = m_scene.addEllipse(QRectF(-7, -7, 14, 14),
                                        QPen(Qt::black), QBrush(QColor(220, 20, 20)));
        item->setZValue(9);
        item->setToolTip(bearId);
        m_bearItems.insert(bearId, item);
        it = m_bearItems.find(bearId);
    }
    (*it)->setPos(pos);
}

void MapWidget::removeBear(const QString &bearId)
{
    if (auto it = m_bearItems.find(bearId); it != m_bearItems.end()) {
        m_scene.removeItem(it.value());
        delete it.value();
        m_bearItems.erase(it);
    }
    m_bearLatLon.remove(bearId);
}

double MapWidget::closestBearDistanceToPerimeter() const
{
    // Bears are supposed to STAY INSIDE the perimeter (it's their allowed/
    // safe zone), so "margin" here means how far a bear still is from
    // crossing OUTWARD. A bear right at the center of the circle has a
    // large positive margin (very safe); a bear right at the boundary has
    // ~0 margin; a bear that has actually left the circle gets a NEGATIVE
    // margin. TrafficLightWidget::setDistanceToPerimeter() already treats
    // <= 0 as Danger (red) unconditionally, so no change is needed there -
    // only this formula needed to flip.
    double smallestMargin = std::numeric_limits<double>::max();
    for (auto *item : m_bearItems) {
        QPointF p = item->pos();
        double distFromCenter = std::hypot(p.x(), p.y());
        double marginBeforeLeavingPerimeter = m_perimeterMeters - distFromCenter;
        smallestMargin = std::min(smallestMargin, marginBeforeLeavingPerimeter);
    }
    return smallestMargin;
}

void MapWidget::resizeEvent(QResizeEvent *event)
{
    QGraphicsView::resizeEvent(event);
    if (!m_scene.itemsBoundingRect().isEmpty())
        fitInView(m_scene.itemsBoundingRect().adjusted(-50, -50, 50, 50), Qt::KeepAspectRatio);
}
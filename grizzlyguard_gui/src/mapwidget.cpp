// ============================================================
// MAPWIDGET.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "mapwidget.h"

#include <QDialog>
#include <QHBoxLayout>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QTimer>
#include <QVBoxLayout>
#include <QDebug>
#include <algorithm>

// ============================================================
// MAP WIDGET
// ============================================================
MapWidget::MapWidget(QWidget *parent) : QGraphicsView(parent)
{
    setScene(&m_scene);
    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    setBackgroundBrush(Qt::black);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setDragMode(QGraphicsView::ScrollHandDrag);

    const QPixmap image(":/images/images/world_map.png");
    if (!image.isNull()) {
        m_background = m_scene.addPixmap(image);
        m_background->setZValue(-100);
        m_scene.setSceneRect(m_background->boundingRect());
    } else {
        qWarning() << "World map resource missing: :/images/images/world_map.png";
        m_scene.setSceneRect(0, 0, 800, 800);
    }

    m_droneItem = m_scene.addEllipse(QRectF(-7, -7, 14, 14),
                                     QPen(Qt::white, 2), QBrush(QColor(30, 100, 255)));
    m_droneItem->setZValue(10);
    m_droneItem->setToolTip("Drone");
    m_droneItem->setVisible(false); // Wait for a real world-frame position.

    m_expandButton = new QPushButton("Expand Map", viewport());
    m_expandButton->setStyleSheet(
        "QPushButton { background: #292929; color: white; padding: 7px 12px; "
        "border-radius: 5px; font-weight: bold; }");
    connect(m_expandButton, &QPushButton::clicked, this, &MapWidget::openExpandedMap);
    QTimer::singleShot(0, this, [this] { fitMapToPanel(); });
}

// ============================================================
// SET WORLD BOUNDS
// ============================================================
void MapWidget::setWorldBounds(double minX, double maxX, double minY, double maxY)
{
    if (maxX <= minX || maxY <= minY)
        return;
    m_minX = minX;
    m_maxX = maxX;
    m_minY = minY;
    m_maxY = maxY;
    updateMarkers();
}

// ============================================================
// PROJECT TO SCENE
// ============================================================
QPointF MapWidget::projectToScene(double x, double y) const
{
    const QRectF r = m_scene.sceneRect();
    const double u = (x - m_minX) / (m_maxX - m_minX);
    const double v = (m_maxY - y) / (m_maxY - m_minY);
    return QPointF(r.left() + u * r.width(), r.top() + v * r.height());
}

// ============================================================
// SET DRONE POSITION
// ============================================================
void MapWidget::setDronePosition(double x, double y, double headingDegrees)
{
    m_dronePosition = QPointF(x, y);
    m_hasDronePosition = true;
    m_droneItem->setPos(projectToScene(x, y));
    m_droneItem->setVisible(true);
    Q_UNUSED(headingDegrees);
}

// ============================================================
// UPSERT BEAR
// ============================================================
void MapWidget::upsertBear(const QString &bearId, double x, double y)
{
    m_bearPositions[bearId] = QPointF(x, y);
    auto it = m_bearItems.find(bearId);
    if (it == m_bearItems.end()) {
        auto *marker = m_scene.addEllipse(QRectF(-7, -7, 14, 14),
                                          QPen(Qt::black, 2), QBrush(QColor(220, 20, 20)));
        marker->setZValue(9);
        marker->setToolTip(bearId);
        m_bearItems.insert(bearId, marker);
        it = m_bearItems.find(bearId);
    }
    it.value()->setPos(projectToScene(x, y));
}

// ============================================================
// REMOVE BEAR
// ============================================================
void MapWidget::removeBear(const QString &bearId)
{
    auto it = m_bearItems.find(bearId);
    if (it != m_bearItems.end()) {
        QGraphicsEllipseItem *marker = it.value();
        m_scene.removeItem(marker);
        delete marker;
        m_bearItems.erase(it);
    }
    m_bearPositions.remove(bearId);
}

// ============================================================
// UPDATE MARKERS
// ============================================================
void MapWidget::updateMarkers()
{
    if (m_hasDronePosition)
        m_droneItem->setPos(projectToScene(m_dronePosition.x(), m_dronePosition.y()));
    for (auto it = m_bearPositions.cbegin(); it != m_bearPositions.cend(); ++it)
        m_bearItems[it.key()]->setPos(projectToScene(it.value().x(), it.value().y()));
}

// ============================================================
// FIT MAP TO PANEL
// ============================================================
void MapWidget::fitMapToPanel()
{
    const QRectF r = m_scene.sceneRect();
    if (r.isEmpty() || viewport()->width() <= 0 || viewport()->height() <= 0)
        return;
    const double sx = static_cast<double>(viewport()->width()) / r.width();
    const double sy = static_cast<double>(viewport()->height()) / r.height();
    resetTransform();
    scale(std::max(sx, sy), std::max(sx, sy));
    centerOn(r.center());
}

// ============================================================
// RESIZE EVENT
// ============================================================
void MapWidget::resizeEvent(QResizeEvent *event)
{
    QGraphicsView::resizeEvent(event);
    fitMapToPanel();
    if (m_expandButton) {
        m_expandButton->adjustSize();
        m_expandButton->move(std::max(0, viewport()->width() - m_expandButton->width() - 10), 10);
    }
}

// ============================================================
// OPEN EXPANDED MAP
// ============================================================
void MapWidget::openExpandedMap()
{
    auto *dialog = new QDialog(this);
    dialog->setWindowTitle("Gazebo World Map");
    dialog->resize(1100, 800);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    auto *layout = new QVBoxLayout(dialog);
    auto *view = new QGraphicsView(&m_scene, dialog);
    view->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    view->setDragMode(QGraphicsView::ScrollHandDrag);
    layout->addWidget(view);

    auto *buttons = new QHBoxLayout();
    auto *fitButton = new QPushButton("Show Entire Map", dialog);
    auto *closeButton = new QPushButton("Close", dialog);
    buttons->addWidget(fitButton);
    buttons->addStretch();
    buttons->addWidget(closeButton);
    layout->addLayout(buttons);

    auto fit = [this, view] { view->fitInView(m_scene.sceneRect(), Qt::KeepAspectRatio); };
    connect(fitButton, &QPushButton::clicked, dialog, fit);
    connect(closeButton, &QPushButton::clicked, dialog, &QDialog::close);
    dialog->show();
    QTimer::singleShot(0, dialog, fit);
}

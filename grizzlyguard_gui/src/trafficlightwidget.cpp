// ============================================================
// TRAFFICLIGHTWIDGET.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "trafficlightwidget.h"

#include <QPainter>

// ============================================================
// TRAFFIC LIGHT WIDGET
// ============================================================
TrafficLightWidget::TrafficLightWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(60, 160);
}

// ============================================================
// SET LEVEL
// ============================================================
void TrafficLightWidget::setLevel(Level level)
{
    if (m_level == level)
        return;
    m_level = level;
    update();
}

// ============================================================
// SET DISTANCE TO PERIMETER
// ============================================================
void TrafficLightWidget::setDistanceToPerimeter(double distanceMeters,
                                                double cautionMeters,
                                                double dangerMeters)
{
    // Compatibility helper; the integration should use setLevel() directly.
    if (distanceMeters <= 0.0) {
        setLevel(Level::Danger);
        return;
    }

    if (distanceMeters <= dangerMeters)
        setLevel(Level::Danger);
    else if (distanceMeters <= cautionMeters)
        setLevel(Level::Caution);
    else
        setLevel(Level::Safe);
}

// ============================================================
// PAINT EVENT
// ============================================================
void TrafficLightWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Housing
    painter.setBrush(QColor(30, 30, 30));
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(rect().adjusted(2, 2, -2, -2), 10, 10);

    const int diameter = qMin(width() - 20, (height() - 20) / 3);
    const int x = (width() - diameter) / 2;

    auto drawLamp = [&](int index, const QColor &onColor, bool isOn) {
        int y = 10 + index * (diameter + 5);
        painter.setBrush(isOn ? onColor : onColor.darker(400));
        painter.drawEllipse(x, y, diameter, diameter);
    };

    drawLamp(0, QColor(220, 40, 40), m_level == Level::Danger);   // red
    drawLamp(1, QColor(240, 200, 30), m_level == Level::Caution); // yellow
    drawLamp(2, QColor(40, 200, 90), m_level == Level::Safe);     // green
}

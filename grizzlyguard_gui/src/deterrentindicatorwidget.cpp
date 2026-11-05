// ============================================================
// DETERRENTINDICATORWIDGET.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "deterrentindicatorwidget.h"

#include <QPainter>
#include <QRadialGradient>

DeterrentIndicatorWidget::DeterrentIndicatorWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(60, 60);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    setToolTip("Deterrent status (/deterrent/status)");
}

void DeterrentIndicatorWidget::setDeployed(bool deployed)
{
    // Always repaint on the first message so the "?" state clears even if
    // the first value is false.
    if (m_hasData && m_deployed == deployed)
        return;
    m_hasData = true;
    m_deployed = deployed;
    update();
}

void DeterrentIndicatorWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Housing (same style as the traffic light)
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(30, 30, 30));
    painter.drawRoundedRect(rect().adjusted(2, 2, -2, -2), 10, 10);

    const int diameter = qMax(10, qMin(width(), height()) - 20);
    const QRectF lamp((width() - diameter) / 2.0, (height() - diameter) / 2.0,
                      diameter, diameter);

    if (!m_hasData) {
        // No data yet: visible grey lamp with a question mark.
        painter.setPen(QPen(QColor(150, 150, 150), 2));
        painter.setBrush(QColor(80, 80, 80));
        painter.drawEllipse(lamp);
        QFont f = painter.font();
        f.setBold(true);
        f.setPixelSize(diameter / 2);
        painter.setFont(f);
        painter.setPen(QColor(200, 200, 200));
        painter.drawText(lamp, Qt::AlignCenter, "?");
        return;
    }

    const QColor on(255, 150, 0);
    if (m_deployed) {
        QRadialGradient glow(lamp.center(), diameter / 2.0);
        glow.setColorAt(0.0, on.lighter(150));
        glow.setColorAt(1.0, on);
        painter.setPen(QPen(on.lighter(130), 2));
        painter.setBrush(glow);
    } else {
        // Off but clearly visible against the housing.
        painter.setPen(QPen(on.darker(160), 2));
        painter.setBrush(QColor(90, 60, 20));
    }
    painter.drawEllipse(lamp);
}
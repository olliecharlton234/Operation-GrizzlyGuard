#include "deterrentindicatorwidget.h"

#include <QPainter>

DeterrentIndicatorWidget::DeterrentIndicatorWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(60, 60);
}

void DeterrentIndicatorWidget::setDeployed(bool deployed)
{
    if (m_deployed == deployed)
        return;
    m_deployed = deployed;
    update();
}

void DeterrentIndicatorWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Housing
    painter.setBrush(QColor(30, 30, 30));
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(rect().adjusted(2, 2, -2, -2), 10, 10);

    const int diameter = qMin(width() - 20, height() - 20);
    const int x = (width() - diameter) / 2;
    const int y = (height() - diameter) / 2;

    QColor onColor(255, 150, 0); // orange - "deterrent active/deployed"
    painter.setBrush(m_deployed ? onColor : onColor.darker(400));
    painter.drawEllipse(x, y, diameter, diameter);
}
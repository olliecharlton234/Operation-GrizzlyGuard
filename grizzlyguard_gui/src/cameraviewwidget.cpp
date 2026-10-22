// ============================================================
// CAMERAVIEWWIDGET.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "cameraviewwidget.h"

#include <QPainter>

// ============================================================
// CAMERA VIEW WIDGET
// ============================================================
CameraViewWidget::CameraViewWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(480, 270); // 16:9
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black);
    setPalette(pal);
}

// ============================================================
// UPDATE FRAME
// ============================================================
void CameraViewWidget::updateFrame(const QImage &frame)
{
    m_currentFrame = frame;
    update();
}

// ============================================================
// SET CONNECTED
// ============================================================
void CameraViewWidget::setConnected(bool connected)
{
    m_connected = connected;
    update();
}

// ============================================================
// PAINT EVENT
// ============================================================
void CameraViewWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    if (!m_currentFrame.isNull()) {
        QImage scaled = m_currentFrame.scaled(size(), Qt::KeepAspectRatio,
                                              Qt::SmoothTransformation);
        int x = (width() - scaled.width()) / 2;
        int y = (height() - scaled.height()) / 2;
        painter.drawImage(x, y, scaled);
    } else {
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter,
                         m_connected ? "Waiting for video..." : "No camera signal");
    }
}

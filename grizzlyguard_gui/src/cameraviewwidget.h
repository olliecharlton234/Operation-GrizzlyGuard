// ============================================================
// CAMERAVIEWWIDGET.H
// Public interface and member declarations
// ============================================================

#ifndef CAMERAVIEWWIDGET_H
#define CAMERAVIEWWIDGET_H


#include <QWidget>
#include <QLabel>
#include <QImage>

// Displays the live camera feed. This widget just paints whatever QImage
// it's given via updateFrame() - it doesn't care where the frame came from.
//
// Wire it up to your video pipeline in one of these ways:
//  - OpenCV: grab cv::Mat in a worker thread, convert to QImage, emit a
//    signal connected to updateFrame() (never touch the widget from another
//    thread directly).
//  - GStreamer: use appsink and pull buffers into QImage on a timer/thread.
//  - QMultimedia: if the drone exposes a UVC/USB camera, QCamera +
//    QVideoWidget can replace this widget entirely.
class CameraViewWidget : public QWidget
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit CameraViewWidget(QWidget *parent = nullptr);

    // -------------------------------------------------------
    // PUBLIC SLOTS
    // -------------------------------------------------------
public slots:
    void updateFrame(const QImage &frame);
    void setConnected(bool connected);

    // -------------------------------------------------------
    // PROTECTED
    // -------------------------------------------------------
protected:
    void paintEvent(QPaintEvent *event) override;

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    QImage m_currentFrame;
    bool m_connected = false;
};




#endif // CAMERAVIEWWIDGET_H

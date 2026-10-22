// ============================================================
// MANUALCONTROLWIDGET.H
// Public interface and member declarations
// ============================================================

#ifndef MANUALCONTROLWIDGET_H
#define MANUALCONTROLWIDGET_H


#include <QWidget>
#include <QImage>
#include <QSet>
#include <QTimer>

class QPushButton;
class CameraViewWidget;
class MapWidget;

// Lives in its own tab - the only tab in the app. Bundles its own live
// camera view AND a live map (drone + bear positions) alongside the
// flight-stick controls, so switching into manual mode never costs you
// situational awareness. Accepts input from keyboard (built in) and/or a
// physical gamepad (see GamepadController, connected externally in
// MainWindow - both drive the same moveRequested signal).
class ManualControlWidget : public QWidget
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit ManualControlWidget(QWidget *parent = nullptr);

    // Exposed so MainWindow can forward drone/bear position updates here
    // too, keeping this tab's map in sync with the Overview tab's map.
    MapWidget *mapWidget() const { return m_mapView; }

    // -------------------------------------------------------
    // PUBLIC SLOTS
    // -------------------------------------------------------
public slots:
    // Feed the same frames here that you feed the Overview tab's camera
    // widget (see MainWindow::updateCameraFrame).
    void updateCameraFrame(const QImage &frame);
    void setCameraConnected(bool connected);

    // -------------------------------------------------------
    // SIGNALS
    // -------------------------------------------------------
signals:
    // Wire these to your drone command interface (MAVLink, custom SDK, etc.)
    // Both keyboard input (handled internally) and an external
    // GamepadController should be connected to feed the same slot on your
    // drone link, so the source of input doesn't matter downstream.
    void moveRequested(double forward, double right, double up, double yawRate);
    void takeoffRequested();
    void landRequested();

    // -------------------------------------------------------
    // PROTECTED
    // -------------------------------------------------------
protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void focusOutEvent(QFocusEvent *event) override;

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    // Held keyboard and button inputs are combined into one velocity command.
    void sendCurrentMovement();
    void bindMovementButton(QPushButton *button, int key);
    QSet<int> m_heldKeys;
    QTimer m_moveTimer;

    CameraViewWidget *m_cameraView;
    MapWidget *m_mapView;
    QPushButton *m_takeoffButton;
    QPushButton *m_landButton;
};


#endif // MANUALCONTROLWIDGET_H

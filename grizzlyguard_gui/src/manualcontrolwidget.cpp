// ============================================================
// MANUALCONTROLWIDGET.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "manualcontrolwidget.h"
#include "cameraviewwidget.h"
#include "mapwidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSplitter>
#include <QPushButton>
#include <QLabel>
#include <QKeyEvent>
#include <QFocusEvent>

// ============================================================
// MANUAL CONTROL WIDGET
// ============================================================
ManualControlWidget::ManualControlWidget(QWidget *parent)
    : QWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus); // so this widget receives key events

    auto *outerLayout = new QVBoxLayout(this);

    auto *splitter = new QSplitter(Qt::Horizontal);

    // Own live camera feed - fed the same frames as the Overview tab's
    // camera widget, via updateCameraFrame().
    m_cameraView = new CameraViewWidget();
    splitter->addWidget(m_cameraView);

    auto *controlsPage = new QWidget();
    auto *mainLayout = new QVBoxLayout(controlsPage);

    auto *hint = new QLabel(
        "Manual mode active. Use W/A/S/D to move, Q/E to yaw, R/F for altitude,\n"
        "or a connected gamepad's sticks. On-screen buttons mirror the same\n"
        "commands for touch/mouse use.");
    hint->setWordWrap(true);
    mainLayout->addWidget(hint);

    // Directional pad
    auto *grid = new QGridLayout();
    auto addBtn = [&](const QString &text, int row, int col) {
        auto *btn = new QPushButton(text);
        btn->setMinimumSize(70, 50);
        grid->addWidget(btn, row, col);
        return btn;
    };

    // Both mouse buttons and keyboard keys drive the same held-input state.
    bindMovementButton(addBtn("Forward", 0, 1), Qt::Key_W);
    bindMovementButton(addBtn("Left", 1, 0), Qt::Key_A);
    bindMovementButton(addBtn("Right", 1, 2), Qt::Key_D);
    bindMovementButton(addBtn("Back", 2, 1), Qt::Key_S);
    bindMovementButton(addBtn("Up", 0, 3), Qt::Key_R);
    bindMovementButton(addBtn("Down", 2, 3), Qt::Key_F);
    bindMovementButton(addBtn("Yaw L", 1, 4), Qt::Key_Q);
    bindMovementButton(addBtn("Yaw R", 1, 5), Qt::Key_E);

    // Refresh commands while inputs are held (20 Hz).
    m_moveTimer.setInterval(50);
    connect(&m_moveTimer, &QTimer::timeout, this, &ManualControlWidget::sendCurrentMovement);

    mainLayout->addLayout(grid);

    // Takeoff / land
    auto *tlLayout = new QHBoxLayout();
    m_takeoffButton = new QPushButton("Takeoff");
    m_landButton = new QPushButton("Land");
    connect(m_takeoffButton, &QPushButton::clicked, this, &ManualControlWidget::takeoffRequested);
    connect(m_landButton, &QPushButton::clicked, this, &ManualControlWidget::landRequested);
    tlLayout->addWidget(m_takeoffButton);
    tlLayout->addWidget(m_landButton);
    mainLayout->addLayout(tlLayout);

    mainLayout->addStretch();

    splitter->addWidget(controlsPage);

    // Own live map - kept in sync with the Overview tab's map via
    // mapWidget() so you can see drone/bear position while flying manually.
    m_mapView = new MapWidget();
    splitter->addWidget(m_mapView);

    splitter->setStretchFactor(0, 2); // camera gets the most room
    splitter->setStretchFactor(1, 1); // controls
    splitter->setStretchFactor(2, 2); // map

    outerLayout->addWidget(splitter);
}

// ============================================================
// UPDATE CAMERA FRAME
// ============================================================
void ManualControlWidget::updateCameraFrame(const QImage &frame)
{
    m_cameraView->updateFrame(frame);
}

// ============================================================
// SET CAMERA CONNECTED
// ============================================================
void ManualControlWidget::setCameraConnected(bool connected)
{
    m_cameraView->setConnected(connected);
}

// ============================================================
// BIND MOVEMENT BUTTON
// ============================================================
void ManualControlWidget::bindMovementButton(QPushButton *button, int key)
{
    button->setFocusPolicy(Qt::NoFocus); // Preserve keyboard focus for WASD.
    connect(button, &QPushButton::pressed, this, [this, key] {
        m_heldKeys.insert(key);
        sendCurrentMovement();
        m_moveTimer.start();
    });
    connect(button, &QPushButton::released, this, [this, key] {
        m_heldKeys.remove(key);
        sendCurrentMovement();
        if (m_heldKeys.isEmpty())
            m_moveTimer.stop();
    });
}

// ============================================================
// SEND CURRENT MOVEMENT
// ============================================================
void ManualControlWidget::sendCurrentMovement()
{
    auto axis = [this](int positive, int negative) -> double {
        return static_cast<double>(m_heldKeys.contains(positive))
             - static_cast<double>(m_heldKeys.contains(negative));
    };
    emit moveRequested(axis(Qt::Key_W, Qt::Key_S),
                       axis(Qt::Key_D, Qt::Key_A),
                       axis(Qt::Key_R, Qt::Key_F),
                       axis(Qt::Key_E, Qt::Key_Q));
}

// ============================================================
// KEY PRESS EVENT
// ============================================================
void ManualControlWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) {
        event->accept();
        return;
    }
    switch (event->key()) {
    case Qt::Key_W: case Qt::Key_A: case Qt::Key_S: case Qt::Key_D:
    case Qt::Key_Q: case Qt::Key_E: case Qt::Key_R: case Qt::Key_F:
        m_heldKeys.insert(event->key());
        sendCurrentMovement();
        m_moveTimer.start();
        event->accept();
        return;
    default:
        QWidget::keyPressEvent(event);
    }
}

// ============================================================
// KEY RELEASE EVENT
// ============================================================
void ManualControlWidget::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) {
        event->accept();
        return;
    }
    if (m_heldKeys.remove(event->key())) {
        sendCurrentMovement();
        if (m_heldKeys.isEmpty())
            m_moveTimer.stop();
        event->accept();
    } else {
        QWidget::keyReleaseEvent(event);
    }
}

// ============================================================
// FOCUS OUT EVENT
// ============================================================
void ManualControlWidget::focusOutEvent(QFocusEvent *event)
{
    m_heldKeys.clear();
    m_moveTimer.stop();
    sendCurrentMovement();
    QWidget::focusOutEvent(event);
}

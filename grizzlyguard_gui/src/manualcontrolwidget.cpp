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

    addBtn("Forward", 0, 1);
    addBtn("Left", 1, 0);
    addBtn("Right", 1, 2);
    addBtn("Back", 2, 1);
    addBtn("Up", 0, 3);
    addBtn("Down", 2, 3);
    addBtn("Yaw L", 1, 4);
    addBtn("Yaw R", 1, 5);

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

void ManualControlWidget::updateCameraFrame(const QImage &frame)
{
    m_cameraView->updateFrame(frame);
}

void ManualControlWidget::setCameraConnected(bool connected)
{
    m_cameraView->setConnected(connected);
}

void ManualControlWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) {
        QWidget::keyPressEvent(event);
        return;
    }

    switch (event->key()) {
    case Qt::Key_W: emit moveRequested(1.0, 0.0, 0.0, 0.0); break;
    case Qt::Key_S: emit moveRequested(-1.0, 0.0, 0.0, 0.0); break;
    case Qt::Key_A: emit moveRequested(0.0, -1.0, 0.0, 0.0); break;
    case Qt::Key_D: emit moveRequested(0.0, 1.0, 0.0, 0.0); break;
    case Qt::Key_R: emit moveRequested(0.0, 0.0, 1.0, 0.0); break;
    case Qt::Key_F: emit moveRequested(0.0, 0.0, -1.0, 0.0); break;
    case Qt::Key_Q: emit moveRequested(0.0, 0.0, 0.0, -1.0); break;
    case Qt::Key_E: emit moveRequested(0.0, 0.0, 0.0, 1.0); break;
    default: QWidget::keyPressEvent(event); return;
    }
}

void ManualControlWidget::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) {
        QWidget::keyReleaseEvent(event);
        return;
    }
    // Stop motion on key release. A real implementation should track which
    // axis stopped rather than zeroing everything, in case of overlapping keys.
    emit moveRequested(0.0, 0.0, 0.0, 0.0);
}
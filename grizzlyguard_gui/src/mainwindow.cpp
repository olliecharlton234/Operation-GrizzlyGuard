#include "mainwindow.h"
#include "cameraviewwidget.h"
#include "mapwidget.h"
#include "telemetrypanel.h"
#include "manualcontrolwidget.h"
#include "gamepadcontroller.h"
#include "perimeteralertdialog.h"
#include "dronebackend.h"

#include <QStackedWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QPushButton>
#include <QDialog>
#include <QLabel>
#include <QMessageBox>
#include <QStatusBar>
#include <QTimer>

namespace {
// The robot does not report a battery level, so the GUI simulates one: it
// starts at 100% and loses 1% every this many seconds.
// 18 seconds per percent = 100% down to 0% in 30 minutes. Change this one
// number to make it drain faster or slower.
constexpr int kBatterySecondsPerPercent = 18;

// Shows a large confirmation box and returns true only if the operator
// presses the confirm button. Used for Manual Mode and Emergency Stop so both
// look the same.
//
// This is a QDialog with its own layout rather than a QMessageBox: QMessageBox
// fixes its size using the default font before any stylesheet is applied, so
// larger text gets clipped. A layout-based dialog sizes itself from the text
// it actually has, so nothing is cut off.
bool confirmLarge(QWidget *parent, const QString &title, const QString &text,
                  const QString &detail, const QString &confirmLabel)
{
    QDialog dialog(parent);
    dialog.setWindowTitle(title);
    dialog.setModal(true);
    dialog.setMinimumWidth(760);
    dialog.setStyleSheet(
        "QLabel#headline { font-size: 26px; font-weight: bold; }"
        "QLabel#detail { font-size: 20px; }"
        "QPushButton { min-width: 240px; min-height: 60px; font-size: 18px; font-weight: bold; }");

    auto *layout = new QVBoxLayout(&dialog);
    layout->setContentsMargins(36, 30, 36, 30);
    layout->setSpacing(20);

    auto *headline = new QLabel(text);
    headline->setObjectName("headline");
    headline->setWordWrap(true);
    layout->addWidget(headline);

    auto *detailLabel = new QLabel(detail);
    detailLabel->setObjectName("detail");
    detailLabel->setWordWrap(true);
    layout->addWidget(detailLabel);

    auto *buttons = new QHBoxLayout();
    buttons->setSpacing(20);
    auto *confirm = new QPushButton(confirmLabel);
    auto *cancel = new QPushButton("Cancel");
    confirm->setAutoDefault(false);
    cancel->setDefault(true); // Enter/Space cancels; confirming takes a deliberate click
    cancel->setFocus();
    buttons->addStretch();
    buttons->addWidget(confirm);
    buttons->addWidget(cancel);
    layout->addLayout(buttons);

    QObject::connect(confirm, &QPushButton::clicked, &dialog, &QDialog::accept);
    QObject::connect(cancel, &QPushButton::clicked, &dialog, &QDialog::reject); // Esc also rejects

    dialog.adjustSize();
    return dialog.exec() == QDialog::Accepted;
}
} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("Bear Drone Monitor");

    auto *central = new QWidget(this);
    setCentralWidget(central);
    auto *mainLayout = new QVBoxLayout(central);

    // Two pages with no tab bar: the only way into Manual Mode is the
    // Manual Mode button, which asks for confirmation first.
    m_pages = new QStackedWidget();
    m_pages->addWidget(buildOverviewTab());

    m_manualControl = new ManualControlWidget();
    m_pages->addWidget(m_manualControl);
    connect(m_pages, &QStackedWidget::currentChanged, this, &MainWindow::onPageChanged);

    // --- The backend adapter: everything drone-related flows through this
    // one object. See dronebackend.h for how to swap in the real thing. ---
    m_alertDialog = new PerimeterAlertDialog(this);

    m_backend = new DroneBackend(this);

    connect(m_backend, &DroneBackend::dronePositionChanged, this, &MainWindow::onDronePositionChanged);
    connect(m_backend, &DroneBackend::bearDetected, this, &MainWindow::onBearDetected);
    connect(m_backend, &DroneBackend::batteryChanged, this, &MainWindow::onBatteryChanged);
    connect(m_backend, &DroneBackend::frameReady, this, [this](const QImage &frame) {
        m_cameraView->updateFrame(frame);
        m_manualControl->updateCameraFrame(frame);
    });
    connect(m_backend, &DroneBackend::droneNameChanged, m_telemetryPanel, &TelemetryPanel::setDroneName);
    connect(m_backend, &DroneBackend::deterrentDeployedChanged, m_telemetryPanel, &TelemetryPanel::setDeterrentDeployed);
    connect(m_backend, &DroneBackend::connectionStatusChanged, this, [this](bool connected) {
        statusBar()->showMessage(connected ? "Connected to simulation"
                                           : "Waiting for simulation to start...");
    });
    connect(m_backend, &DroneBackend::videoConnectionChanged, this, [this](bool connected) {
        // Separate from telemetry connection - video comes from a different
        // process (the game engine) and may connect at a different time.
        m_cameraView->setConnected(connected);
        m_manualControl->setCameraConnected(connected);
    });

    // --- Keyboard (via ManualControlWidget) and gamepad both feed the same
    // DroneBackend commands - the backend doesn't know or care which one
    // triggered the call. ---
    connect(m_manualControl, &ManualControlWidget::moveRequested, m_backend, &DroneBackend::sendMove);
    connect(m_manualControl, &ManualControlWidget::takeoffRequested, m_backend, &DroneBackend::sendTakeoff);
    connect(m_manualControl, &ManualControlWidget::landRequested, m_backend, &DroneBackend::sendLand);

    m_gamepad = new GamepadController(this);
    connect(m_gamepad, &GamepadController::moveRequested, m_backend, &DroneBackend::sendMove);
    connect(m_gamepad, &GamepadController::takeoffRequested, m_backend, &DroneBackend::sendTakeoff);
    connect(m_gamepad, &GamepadController::landRequested, m_backend, &DroneBackend::sendLand);
    connect(m_gamepad, &GamepadController::connectedChanged, this, [this](bool connected) {
        statusBar()->showMessage(connected ? "Gamepad connected" : "Gamepad disconnected", 3000);
    });
    m_gamepad->openFirstAvailable();

    mainLayout->addWidget(m_pages, /*stretch=*/1);

    // --- Always-visible control bar, outside the tabs ---
    auto *controlBar = new QHBoxLayout();

    // Manual Mode button: the only way into Manual Mode. Pressing it asks the
    // operator to confirm (step 2), then switches to the Manual Control page,
    // which engages manual mode. Pressing it again returns to the Overview
    // page, which disengages it. It stays in sync via onPageChanged().
    m_manualModeButton = new QPushButton("Manual Mode");
    m_manualModeButton->setCheckable(true);
    m_manualModeButton->setStyleSheet(
        "QPushButton { padding: 10px 20px; border-radius: 6px; } "
        "QPushButton:checked { background-color: #1e64ff; color: white; font-weight: bold; }");
    connect(m_manualModeButton, &QPushButton::clicked, this, &MainWindow::onManualModeButtonClicked);
    controlBar->addWidget(m_manualModeButton);

    controlBar->addStretch();

    m_rthButton = new QPushButton("Return to Home");
    connect(m_rthButton, &QPushButton::clicked, this, &MainWindow::onReturnToHome);
    controlBar->addWidget(m_rthButton);

    m_emergencyStopButton = new QPushButton("EMERGENCY STOP");
    m_emergencyStopButton->setStyleSheet(
        "QPushButton { background-color: #c62828; color: white; font-weight: bold; "
        "padding: 10px 20px; border-radius: 6px; } "
        "QPushButton:pressed { background-color: #8e0000; }");
    connect(m_emergencyStopButton, &QPushButton::clicked, this, &MainWindow::onEmergencyStop);
    controlBar->addWidget(m_emergencyStopButton);

    mainLayout->addLayout(controlBar);

    statusBar()->showMessage("Waiting for simulation to start...");

    m_mapView->setHomePosition(-33.8688, 151.2093); // TODO: use real home/launch coordinates
    m_mapView->setPerimeterRadius(300.0);
    m_manualControl->mapWidget()->setHomePosition(-33.8688, 151.2093);
    m_manualControl->mapWidget()->setPerimeterRadius(300.0);

    // --- Simulated battery (no real battery data exists for the robot) ---
    onBatteryChanged(m_batteryPercent); // show 100% straight away
    auto *batteryTimer = new QTimer(this);
    connect(batteryTimer, &QTimer::timeout, this, [this]() {
        if (m_batteryPercent > 0)
            onBatteryChanged(--m_batteryPercent);
    });
    batteryTimer->start(kBatterySecondsPerPercent * 1000);

    m_backend->start();
}

QWidget *MainWindow::buildOverviewTab()
{
    auto *page = new QWidget();
    auto *layout = new QHBoxLayout(page);

    auto *splitter = new QSplitter(Qt::Horizontal);

    m_cameraView = new CameraViewWidget();
    splitter->addWidget(m_cameraView);

    auto *rightSide = new QWidget();
    auto *rightLayout = new QHBoxLayout(rightSide);
    m_mapView = new MapWidget();
    m_telemetryPanel = new TelemetryPanel();
    rightLayout->addWidget(m_mapView, /*stretch=*/3);
    rightLayout->addWidget(m_telemetryPanel, /*stretch=*/1);
    splitter->addWidget(rightSide);

    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);
    layout->addWidget(splitter);

    return page;
}

void MainWindow::onDronePositionChanged(double latitude, double longitude)
{
    m_mapView->setDronePosition(latitude, longitude);
    m_manualControl->mapWidget()->setDronePosition(latitude, longitude);
    recomputePerimeterStatus();
}

void MainWindow::onBearDetected(const QString &bearId, double latitude, double longitude)
{
    m_mapView->upsertBear(bearId, latitude, longitude);
    m_manualControl->mapWidget()->upsertBear(bearId, latitude, longitude);
    recomputePerimeterStatus();
}

void MainWindow::onBatteryChanged(int percent)
{
    m_telemetryPanel->setBatteryPercent(percent);
}

void MainWindow::recomputePerimeterStatus()
{
    double distance = m_mapView->closestBearDistanceToPerimeter();
    m_telemetryPanel->setPerimeterDistance(distance);

    // distance <= 0 means a bear has left the perimeter (see
    // MapWidget::closestBearDistanceToPerimeter()).
    if (!m_breachActive && distance <= 0.0) {
        // Newly breached: pop the emergency alert up once.
        m_breachActive = true;
        m_alertDialog->showBreach(-distance);
    } else if (m_breachActive && distance <= 0.0) {
        // Still outside: keep the distance text current, but don't re-open
        // the alert if the operator already acknowledged it.
        m_alertDialog->updateDistance(-distance);
    } else if (m_breachActive && distance > 5.0) {
        // Back safely inside (5 m margin stops flicker at the boundary):
        // close the alert and re-arm it for the next breach.
        m_breachActive = false;
        m_alertDialog->hide();
    }
}

void MainWindow::onEmergencyStop()
{
    // NOTE: this is a "stop and land safely" command, NOT an instant motor
    // cut - killing power mid-flight would just make the drone fall. It
    // cancels whatever the drone is currently doing (manual input, mission,
    // etc.) and commands an immediate landing. See onEmergencyStopRequested()
    // in simulationworker.cpp for where that behavior actually gets
    // implemented once real drone control exists.
    if (confirmLarge(this, "Confirm Emergency Stop",
                     "Trigger an emergency stop?",
                     "This will stop the drone's current action and command it to land immediately.",
                     "Emergency Stop")) {
        statusBar()->showMessage("EMERGENCY STOP - drone landing", 5000);
        m_backend->sendEmergencyStop();
    }
}

void MainWindow::onReturnToHome()
{
    statusBar()->showMessage("Return to Home commanded", 5000);
    m_backend->sendReturnToHome();
}

void MainWindow::onManualModeButtonClicked(bool checked)
{
    if (!checked) {
        // Leaving manual mode needs no confirmation.
        m_pages->setCurrentIndex(0); // Overview
        return;
    }

    // Step 2 of 2: the operator must confirm before manual mode engages.
    if (confirmLarge(this, "Enter Manual Mode",
                     "You are about to take manual control of the drone.",
                     "Autonomous behaviour will stop, and the drone will respond "
                     "only to your keyboard or gamepad commands.",
                     "Enter Manual Mode")) {
        // Switching pages fires onPageChanged(), which engages manual mode.
        m_pages->setCurrentWidget(m_manualControl);
    } else {
        m_manualModeButton->setChecked(false); // cancelled: undo the button press
    }
}

void MainWindow::onPageChanged(int index)
{
    const bool manualActive = (m_pages->widget(index) == m_manualControl);
    if (m_manualModeButton)
        m_manualModeButton->setChecked(manualActive); // does not re-trigger clicked
    if (manualActive)
        m_manualControl->setFocus();
    m_backend->setManualModeActive(manualActive);
    statusBar()->showMessage(manualActive ? "Manual mode engaged" : "Manual mode disengaged");
}
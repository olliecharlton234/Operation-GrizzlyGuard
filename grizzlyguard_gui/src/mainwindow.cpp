#include "mainwindow.h"
#include "cameraviewwidget.h"
#include "mapwidget.h"
#include "telemetrypanel.h"
#include "manualcontrolwidget.h"
#include "gamepadcontroller.h"
#include "dronebackend.h"

#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QPushButton>
#include <QMessageBox>
#include <QStatusBar>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("Bear Drone Monitor");

    auto *central = new QWidget(this);
    setCentralWidget(central);
    auto *mainLayout = new QVBoxLayout(central);

    m_tabs = new QTabWidget();
    m_tabs->addTab(buildOverviewTab(), "Overview");

    m_manualControl = new ManualControlWidget();
    m_tabs->addTab(m_manualControl, "Manual Control");
    connect(m_tabs, &QTabWidget::currentChanged, this, &MainWindow::onTabChanged);

    // --- The backend adapter: everything drone-related flows through this
    // one object. See dronebackend.h for how to swap in the real thing. ---
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

    mainLayout->addWidget(m_tabs, /*stretch=*/1);

    // --- Always-visible control bar, outside the tabs ---
    auto *controlBar = new QHBoxLayout();
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
}

void MainWindow::onEmergencyStop()
{
    // NOTE: this is a "stop and land safely" command, NOT an instant motor
    // cut - killing power mid-flight would just make the drone fall. It
    // cancels whatever the drone is currently doing (manual input, mission,
    // etc.) and commands an immediate landing. See onEmergencyStopRequested()
    // in simulationworker.cpp for where that behavior actually gets
    // implemented once real drone control exists.
    auto reply = QMessageBox::warning(this, "Confirm Emergency Stop",
                                      "This will stop the drone's current action and "
                                      "command it to land immediately. Continue?",
                                      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        statusBar()->showMessage("EMERGENCY STOP - drone landing", 5000);
        m_backend->sendEmergencyStop();
    }
}

void MainWindow::onReturnToHome()
{
    statusBar()->showMessage("Return to Home commanded", 5000);
    m_backend->sendReturnToHome();
}

void MainWindow::onTabChanged(int index)
{
    const bool manualActive = (m_tabs->widget(index) == m_manualControl);
    if (manualActive)
        m_manualControl->setFocus();
    m_backend->setManualModeActive(manualActive);
    statusBar()->showMessage(manualActive ? "Manual mode engaged" : "Manual mode disengaged");
}
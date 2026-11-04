// ============================================================
// MAINWINDOW.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

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
#include <QProcess>
#include <QDebug>

namespace {

// Shows a large confirmation box and returns true only if the operator
// presses the confirm button. Used for Manual Mode and Emergency Stop so both
// look the same.
//
// This is a QDialog with its own layout rather than a QMessageBox: QMessageBox
// fixes its size using the default font before any stylesheet is applied, so
// larger text gets clipped. A layout-based dialog sizes itself from the text
// it actually has, so nothing is cut off.

// ============================================================
// CONFIRM LARGE
// ============================================================
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

// ============================================================
// MAIN WINDOW
// ============================================================
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
    connect(m_backend, &DroneBackend::bearLost, this, [this](const QString &id) {
        m_mapView->removeBear(id);
        m_manualControl->mapWidget()->removeBear(id);
    });
    connect(m_backend, &DroneBackend::frameReady, this, [this](const QImage &frame) {
        m_cameraView->updateFrame(frame);
        m_manualControl->updateCameraFrame(frame);
    });
    connect(m_backend, &DroneBackend::droneNameChanged, m_telemetryPanel, &TelemetryPanel::setDroneName);
    connect(m_backend, &DroneBackend::deterrentDeployedChanged, m_telemetryPanel, &TelemetryPanel::setDeterrentDeployed);
    connect(m_backend, &DroneBackend::perimeterStatusChanged, m_telemetryPanel, &TelemetryPanel::setPerimeterStatus);
    connect(m_backend, &DroneBackend::perimeterAlertReceived, this, [this](const QString &message) {
        statusBar()->showMessage(QString("Perimeter alert: %1").arg(message), 10000);
        if (message.startsWith("BREACH", Qt::CaseInsensitive)) {
            bool ok = false;
            const double metres = message.section(':', 1, 1).toDouble(&ok);
            m_alertDialog->showBreach(ok ? metres : 0.0);
        }
    });
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

    // Screenshot bounds are approximate; calibrate before displaying world poses.
    m_mapView->setWorldBounds(-195, 209, -126, 269);
    m_manualControl->mapWidget()->setWorldBounds(-195, 209, -126, 269);

    // --- Existing movement bridge; requires the correct Gazebo model topic ---
    bridgeProcess = new QProcess(this);

    connect(bridgeProcess, &QProcess::errorOccurred,
            this, [](QProcess::ProcessError error) {
        qWarning() << "Gazebo bridge error:" << error;
    });

    connect(bridgeProcess, &QProcess::readyReadStandardError,
            this, [this]() {
        qWarning() << "Gazebo bridge:"
                   << bridgeProcess->readAllStandardError();
    });

    bridgeProcess->start(
        "ros2",
        QStringList{
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "/model/parrot1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "--ros-args",
            "-r",
            "/model/parrot1/cmd_vel:=/cmd_vel"
        }
    );

    // --- Start existing drone backend ---
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

// ============================================================
// ON DRONE POSITION CHANGED
// ============================================================
void MainWindow::onDronePositionChanged(double x, double y)
{
    m_mapView->setDronePosition(x, y);
    m_manualControl->mapWidget()->setDronePosition(x, y);
}

// ============================================================
// ON BEAR DETECTED
// ============================================================
void MainWindow::onBearDetected(const QString &bearId, double x, double y)
{
    m_mapView->upsertBear(bearId, x, y);
    m_manualControl->mapWidget()->upsertBear(bearId, x, y);
}

// ============================================================
// ON EMERGENCY STOP
// ============================================================
void MainWindow::onEmergencyStop()
{
    // The real emergency-landing interface has not been agreed yet.
    if (confirmLarge(this, "Confirm Emergency Stop",
                     "Trigger an emergency stop?",
                     "This command is not connected to a landing controller yet.",
                     "Emergency Stop")) {
        statusBar()->showMessage("Emergency landing is awaiting integration", 5000);
        m_backend->sendEmergencyStop();
    }
}

// ============================================================
// ON RETURN TO HOME
// ============================================================
void MainWindow::onReturnToHome()
{
    statusBar()->showMessage("Return to Home is awaiting integration", 5000);
    m_backend->sendReturnToHome();
}

// ============================================================
// ON MANUAL MODE BUTTON CLICKED
// ============================================================
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

// ============================================================
// ON PAGE CHANGED
// ============================================================
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

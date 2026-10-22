// ============================================================
// MAINWINDOW.CPP
// Main application window and GUI coordination
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


// ============================================================
// CONFIRMATION DIALOG
// ============================================================

namespace {

// Displays a confirmation dialog before entering manual mode
// or triggering an emergency stop.

bool confirmLarge(QWidget *parent,
                  const QString &title,
                  const QString &text,
                  const QString &detail,
                  const QString &confirmLabel)
{
    QDialog dialog(parent);

    dialog.setWindowTitle(title);
    dialog.setModal(true);
    dialog.setMinimumWidth(760);

    dialog.setStyleSheet(
        "QLabel#headline {"
        "font-size: 26px;"
        "font-weight: bold;"
        "}"
        "QLabel#detail {"
        "font-size: 20px;"
        "}"
        "QPushButton {"
        "min-width: 240px;"
        "min-height: 60px;"
        "font-size: 18px;"
        "font-weight: bold;"
        "}"
    );

    auto *layout = new QVBoxLayout(&dialog);

    layout->setContentsMargins(36, 30, 36, 30);
    layout->setSpacing(20);

    // Confirmation heading
    auto *headline = new QLabel(text);
    headline->setObjectName("headline");
    headline->setWordWrap(true);

    layout->addWidget(headline);

    // Additional confirmation details
    auto *detailLabel = new QLabel(detail);
    detailLabel->setObjectName("detail");
    detailLabel->setWordWrap(true);

    layout->addWidget(detailLabel);

    // Confirmation buttons
    auto *buttons = new QHBoxLayout();
    buttons->setSpacing(20);

    auto *confirm = new QPushButton(confirmLabel);
    auto *cancel = new QPushButton("Cancel");

    confirm->setAutoDefault(false);
    cancel->setDefault(true);
    cancel->setFocus();

    buttons->addStretch();
    buttons->addWidget(confirm);
    buttons->addWidget(cancel);

    layout->addLayout(buttons);

    QObject::connect(
        confirm,
        &QPushButton::clicked,
        &dialog,
        &QDialog::accept
    );

    QObject::connect(
        cancel,
        &QPushButton::clicked,
        &dialog,
        &QDialog::reject
    );

    dialog.adjustSize();

    return dialog.exec() == QDialog::Accepted;
}

} // namespace


// ============================================================
// MAIN WINDOW CONSTRUCTOR
// ============================================================

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("Bear Drone Monitor");


    // ========================================================
    // MAIN WINDOW LAYOUT
    // ========================================================

    auto *central = new QWidget(this);
    setCentralWidget(central);

    auto *mainLayout = new QVBoxLayout(central);


    // ========================================================
    // APPLICATION PAGES
    // ========================================================

    // The application contains an Overview page and a
    // Manual Control page. Manual Mode is accessed through
    // the confirmation button rather than a tab bar.

    m_pages = new QStackedWidget();

    m_pages->addWidget(buildOverviewTab());

    m_manualControl = new ManualControlWidget();
    m_pages->addWidget(m_manualControl);

    connect(
        m_pages,
        &QStackedWidget::currentChanged,
        this,
        &MainWindow::onPageChanged
    );


    // ========================================================
    // BACKEND INITIALISATION
    // ========================================================

    // DroneBackend manages communication between the GUI
    // and the ROS 2 simulation worker.

    m_alertDialog = new PerimeterAlertDialog(this);

    m_backend = new DroneBackend(this);


    // ========================================================
    // DRONE POSITION AND BEAR TRACKING
    // ========================================================

    connect(
        m_backend,
        &DroneBackend::dronePositionChanged,
        this,
        &MainWindow::onDronePositionChanged
    );

    connect(
        m_backend,
        &DroneBackend::bearDetected,
        this,
        &MainWindow::onBearDetected
    );

    // Remove bear markers when tracking is lost
    connect(
        m_backend,
        &DroneBackend::bearLost,
        this,
        [this](const QString &id) {

            m_mapView->removeBear(id);

            m_manualControl->mapWidget()->removeBear(id);
        }
    );


    // ========================================================
    // CAMERA FEED
    // ========================================================

    // Incoming camera frames are displayed on both the
    // Overview and Manual Control pages.

    connect(
        m_backend,
        &DroneBackend::frameReady,
        this,
        [this](const QImage &frame) {

            m_cameraView->updateFrame(frame);

            m_manualControl->updateCameraFrame(frame);
        }
    );


    // ========================================================
    // TELEMETRY CONNECTIONS
    // ========================================================

    connect(
        m_backend,
        &DroneBackend::droneNameChanged,
        m_telemetryPanel,
        &TelemetryPanel::setDroneName
    );

    connect(
        m_backend,
        &DroneBackend::deterrentDeployedChanged,
        m_telemetryPanel,
        &TelemetryPanel::setDeterrentDeployed
    );


    // ========================================================
    // CONNECTION STATUS
    // ========================================================

    connect(
        m_backend,
        &DroneBackend::connectionStatusChanged,
        this,
        [this](bool connected) {

            statusBar()->showMessage(
                connected
                    ? "Connected to simulation"
                    : "Waiting for simulation to start..."
            );
        }
    );

    connect(
        m_backend,
        &DroneBackend::videoConnectionChanged,
        this,
        [this](bool connected) {

            m_cameraView->setConnected(connected);

            m_manualControl->setCameraConnected(connected);
        }
    );


    // ========================================================
    // MANUAL MOVEMENT CONTROLS
    // ========================================================

    // Keyboard and on-screen movement requests are passed
    // to DroneBackend for ROS 2 publishing.

    connect(
        m_manualControl,
        &ManualControlWidget::moveRequested,
        m_backend,
        &DroneBackend::sendMove
    );

    connect(
        m_manualControl,
        &ManualControlWidget::takeoffRequested,
        m_backend,
        &DroneBackend::sendTakeoff
    );

    connect(
        m_manualControl,
        &ManualControlWidget::landRequested,
        m_backend,
        &DroneBackend::sendLand
    );


    // ========================================================
    // GAMEPAD CONTROLLER
    // ========================================================

    m_gamepad = new GamepadController(this);

    connect(
        m_gamepad,
        &GamepadController::moveRequested,
        m_backend,
        &DroneBackend::sendMove
    );

    connect(
        m_gamepad,
        &GamepadController::takeoffRequested,
        m_backend,
        &DroneBackend::sendTakeoff
    );

    connect(
        m_gamepad,
        &GamepadController::landRequested,
        m_backend,
        &DroneBackend::sendLand
    );

    connect(
        m_gamepad,
        &GamepadController::connectedChanged,
        this,
        [this](bool connected) {

            statusBar()->showMessage(
                connected
                    ? "Gamepad connected"
                    : "Gamepad disconnected",
                3000
            );
        }
    );

    m_gamepad->openFirstAvailable();


    // ========================================================
    // ADD APPLICATION PAGES TO LAYOUT
    // ========================================================

    mainLayout->addWidget(m_pages, 1);


    // ========================================================
    // CONTROL BAR
    // ========================================================

    auto *controlBar = new QHBoxLayout();


    // ========================================================
    // MANUAL MODE BUTTON
    // ========================================================

    m_manualModeButton = new QPushButton("Manual Mode");

    m_manualModeButton->setCheckable(true);

    m_manualModeButton->setStyleSheet(
        "QPushButton {"
        "padding: 10px 20px;"
        "border-radius: 6px;"
        "}"
        "QPushButton:checked {"
        "background-color: #1e64ff;"
        "color: white;"
        "font-weight: bold;"
        "}"
    );

    connect(
        m_manualModeButton,
        &QPushButton::clicked,
        this,
        &MainWindow::onManualModeButtonClicked
    );

    controlBar->addWidget(m_manualModeButton);

    controlBar->addStretch();


    // ========================================================
    // RETURN TO HOME BUTTON
    // ========================================================

    m_rthButton = new QPushButton("Return to Home");

    connect(
        m_rthButton,
        &QPushButton::clicked,
        this,
        &MainWindow::onReturnToHome
    );

    controlBar->addWidget(m_rthButton);


    // ========================================================
    // EMERGENCY STOP BUTTON
    // ========================================================

    m_emergencyStopButton =
        new QPushButton("EMERGENCY STOP");

    m_emergencyStopButton->setStyleSheet(
        "QPushButton {"
        "background-color: #c62828;"
        "color: white;"
        "font-weight: bold;"
        "padding: 10px 20px;"
        "border-radius: 6px;"
        "}"
        "QPushButton:pressed {"
        "background-color: #8e0000;"
        "}"
    );

    connect(
        m_emergencyStopButton,
        &QPushButton::clicked,
        this,
        &MainWindow::onEmergencyStop
    );

    controlBar->addWidget(m_emergencyStopButton);

    mainLayout->addLayout(controlBar);


    // ========================================================
    // INITIAL CONNECTION STATUS
    // ========================================================

    statusBar()->showMessage(
        "Waiting for simulation to start..."
    );


    // ========================================================
    // GAZEBO WORLD MAP
    // ========================================================

    // Approximate Yellowstone screenshot boundaries.
    // These values will be calibrated using Gazebo
    // world coordinates during integration.

    m_mapView->setWorldBounds(
        -195, 209, -126, 269
    );

    m_manualControl->mapWidget()->setWorldBounds(
        -195, 209, -126, 269
    );


    // ========================================================
    // SIMULATED BATTERY
    // ========================================================

    // Battery starts at 100% and decreases by 1% every
    // 18 seconds.
    //
    // The battery is simulated locally by the GUI and does
    // not require any ROS 2 or Gazebo battery information.

    m_telemetryPanel->setBatteryPercent(100);

    auto *batteryTimer = new QTimer(this);

    connect(
        batteryTimer,
        &QTimer::timeout,
        this,
        [this, batteryPercent = 100]() mutable {

            if (batteryPercent > 0) {

                --batteryPercent;

                m_telemetryPanel->setBatteryPercent(
                    batteryPercent
                );
            }
        }
    );

    batteryTimer->start(18000);


    // ========================================================
    // ROS 2 TO GAZEBO MOVEMENT BRIDGE
    // ========================================================

    // Starts the movement bridge when the GUI opens.
    //
    // This assumes the simulation uses the Gazebo topic:
    // /model/parrot1/cmd_vel
    //
    // The ROS 2 command topic remains:
    // /cmd_vel

    bridgeProcess = new QProcess(this);

    connect(
        bridgeProcess,
        &QProcess::errorOccurred,
        this,
        [](QProcess::ProcessError error) {

            qWarning()
                << "Gazebo bridge error:"
                << error;
        }
    );

    connect(
        bridgeProcess,
        &QProcess::readyReadStandardError,
        this,
        [this]() {

            qWarning()
                << "Gazebo bridge:"
                << bridgeProcess->readAllStandardError();
        }
    );

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


    // ========================================================
    // START DRONE BACKEND
    // ========================================================

    // Start the existing ROS 2 backend after initialising
    // the GUI components and bridge.

    m_backend->start();
}


// ============================================================
// BUILD OVERVIEW PAGE
// ============================================================

QWidget *MainWindow::buildOverviewTab()
{
    auto *page = new QWidget();

    auto *layout = new QHBoxLayout(page);

    auto *splitter = new QSplitter(Qt::Horizontal);


    // ========================================================
    // CAMERA VIEW
    // ========================================================

    m_cameraView = new CameraViewWidget();

    splitter->addWidget(m_cameraView);


    // ========================================================
    // MAP AND TELEMETRY
    // ========================================================

    auto *rightSide = new QWidget();

    auto *rightLayout = new QHBoxLayout(rightSide);

    m_mapView = new MapWidget();

    m_telemetryPanel = new TelemetryPanel();

    rightLayout->addWidget(m_mapView, 3);

    rightLayout->addWidget(m_telemetryPanel, 1);

    splitter->addWidget(rightSide);


    // ========================================================
    // SPLITTER CONFIGURATION
    // ========================================================

    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);

    layout->addWidget(splitter);

    return page;
}


// ============================================================
// UPDATE DRONE POSITION
// ============================================================

void MainWindow::onDronePositionChanged(
    double x,
    double y)
{
    // Update both map displays using Gazebo coordinates.

    m_mapView->setDronePosition(x, y);

    m_manualControl->mapWidget()->setDronePosition(
        x, y
    );
}


// ============================================================
// UPDATE BEAR POSITION
// ============================================================

void MainWindow::onBearDetected(
    const QString &bearId,
    double x,
    double y)
{
    // Update detected bears on both maps.

    m_mapView->upsertBear(
        bearId, x, y
    );

    m_manualControl->mapWidget()->upsertBear(
        bearId, x, y
    );
}


// ============================================================
// EMERGENCY STOP
// ============================================================

void MainWindow::onEmergencyStop()
{
    // Emergency landing is awaiting integration with
    // the simulation team's flight controller.

    if (confirmLarge(
            this,
            "Confirm Emergency Stop",
            "Trigger an emergency stop?",
            "This command is not connected to a landing controller yet.",
            "Emergency Stop"))
    {
        statusBar()->showMessage(
            "Emergency landing is awaiting integration",
            5000
        );

        m_backend->sendEmergencyStop();
    }
}


// ============================================================
// RETURN TO HOME
// ============================================================

void MainWindow::onReturnToHome()
{
    // Return to Home is awaiting integration with
    // the autonomous navigation system.

    statusBar()->showMessage(
        "Return to Home is awaiting integration",
        5000
    );

    m_backend->sendReturnToHome();
}


// ============================================================
// MANUAL MODE BUTTON
// ============================================================

void MainWindow::onManualModeButtonClicked(bool checked)
{
    if (!checked) {

        // Return to Overview when Manual Mode is disabled.

        m_pages->setCurrentIndex(0);

        return;
    }

    // Require operator confirmation before entering
    // Manual Control.

    if (confirmLarge(
            this,
            "Enter Manual Mode",
            "You are about to take manual control of the drone.",
            "Autonomous behaviour will stop, and the drone will respond "
            "only to your keyboard or gamepad commands.",
            "Enter Manual Mode"))
    {
        m_pages->setCurrentWidget(
            m_manualControl
        );
    }
    else {

        // Restore button state if the operator cancels.

        m_manualModeButton->setChecked(false);
    }
}


// ============================================================
// PAGE CHANGED
// ============================================================

void MainWindow::onPageChanged(int index)
{
    const bool manualActive =
        (m_pages->widget(index) == m_manualControl);

    // Synchronise Manual Mode button state.

    if (m_manualModeButton) {

        m_manualModeButton->setChecked(
            manualActive
        );
    }

    // Give keyboard focus to Manual Control.

    if (manualActive) {

        m_manualControl->setFocus();
    }

    // Inform the ROS 2 backend of the selected mode.

    m_backend->setManualModeActive(
        manualActive
    );

    statusBar()->showMessage(
        manualActive
            ? "Manual mode engaged"
            : "Manual mode disengaged"
    );
}

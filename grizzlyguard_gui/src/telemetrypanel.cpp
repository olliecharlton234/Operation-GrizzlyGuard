// ============================================================
// TELEMETRYPANEL.CPP
// Drone telemetry and status display
// ============================================================

#include "telemetrypanel.h"
#include "trafficlightwidget.h"
#include "deterrentindicatorwidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QProgressBar>
#include <QFont>
#include <QtGlobal>


// ============================================================
// CONSTRUCTOR
// ============================================================

TelemetryPanel::TelemetryPanel(QWidget *parent)
    : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);


    // ========================================================
    // DRONE IDENTIFICATION
    // ========================================================

    m_nameLabel = new QLabel("Drone: --");

    QFont nameFont = m_nameLabel->font();

    nameFont.setPointSize(nameFont.pointSize() + 2);
    nameFont.setBold(true);

    m_nameLabel->setFont(nameFont);

    layout->addWidget(m_nameLabel);


    // ========================================================
    // BATTERY DISPLAY
    // ========================================================

    // Battery percentage is simulated locally by MainWindow.
    // No physical or Gazebo battery data is required.

    auto *batteryBox = new QGroupBox("Battery");
    auto *batteryLayout = new QVBoxLayout(batteryBox);

    m_batteryLabel = new QLabel("100%");

    m_batteryBar = new QProgressBar();
    m_batteryBar->setRange(0, 100);
    m_batteryBar->setValue(100);
    m_batteryBar->setTextVisible(false);

    m_batteryBar->setStyleSheet(
        "QProgressBar {"
        "border: 1px solid #777;"
        "border-radius: 5px;"
        "background-color: #333;"
        "height: 18px;"
        "}"
        "QProgressBar::chunk {"
        "background-color: #4caf50;"
        "border-radius: 4px;"
        "}"
    );

    batteryLayout->addWidget(m_batteryLabel);
    batteryLayout->addWidget(m_batteryBar);

    layout->addWidget(batteryBox);


    // ========================================================
    // PERIMETER STATUS
    // ========================================================

    // Perimeter calculations are handled by the separate
    // perimeter system and will be integrated through ROS 2.

    auto *perimeterBox =
        new QGroupBox("Bear Perimeter Status");

    auto *perimeterLayout =
        new QHBoxLayout(perimeterBox);

    m_trafficLight = new TrafficLightWidget();

    perimeterLayout->addWidget(m_trafficLight);
    perimeterLayout->addStretch();

    layout->addWidget(perimeterBox);


    // ========================================================
    // DETERRENT STATUS
    // ========================================================

    // Deterrent information will be provided by the team's
    // decision-making system through ROS 2.

    auto *deterrentBox = new QGroupBox("Deterrent");
    auto *deterrentLayout =
        new QHBoxLayout(deterrentBox);

    m_deterrentIndicator =
        new DeterrentIndicatorWidget();

    m_deterrentLabel = new QLabel("Awaiting data");

    deterrentLayout->addWidget(m_deterrentIndicator);
    deterrentLayout->addWidget(m_deterrentLabel);
    deterrentLayout->addStretch();

    layout->addWidget(deterrentBox);

    layout->addStretch();
}


// ============================================================
// UPDATE DRONE NAME
// ============================================================

void TelemetryPanel::setDroneName(const QString &name)
{
    m_nameLabel->setText(
        QString("Drone: %1").arg(name)
    );
}


// ============================================================
// UPDATE BATTERY
// ============================================================

void TelemetryPanel::setBatteryPercent(int percent)
{
    // Keep battery values within the valid range
    percent = qBound(0, percent, 100);

    // Update displayed percentage
    m_batteryLabel->setText(
        QString("%1%").arg(percent)
    );

    // Update battery progress bar
    m_batteryBar->setValue(percent);

    // Change colour based on battery level
    QString batteryColor;

    if (percent > 50) {
        batteryColor = "#4caf50";  // Green
    }
    else if (percent > 20) {
        batteryColor = "#ff9800";  // Orange
    }
    else {
        batteryColor = "#f44336";  // Red
    }

    m_batteryBar->setStyleSheet(
        QString(
            "QProgressBar {"
            "border: 1px solid #777;"
            "border-radius: 5px;"
            "background-color: #333;"
            "height: 18px;"
            "}"
            "QProgressBar::chunk {"
            "background-color: %1;"
            "border-radius: 4px;"
            "}"
        ).arg(batteryColor)
    );
}


// ============================================================
// UPDATE PERIMETER DISTANCE
// ============================================================

void TelemetryPanel::setPerimeterDistance(
    double distanceMeters)
{
    m_trafficLight->setDistanceToPerimeter(
        distanceMeters
    );
}


// ============================================================
// UPDATE DETERRENT STATUS
// ============================================================

void TelemetryPanel::setDeterrentDeployed(bool deployed)
{
    m_deterrentIndicator->setDeployed(deployed);

    m_deterrentLabel->setText(
        deployed ? "Deployed" : "Not deployed"
    );
}

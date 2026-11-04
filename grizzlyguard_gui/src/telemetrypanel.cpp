// ============================================================
// TELEMETRYPANEL.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "telemetrypanel.h"
#include "trafficlightwidget.h"
#include "deterrentindicatorwidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QFont>
#include <QProgressBar>
#include <QTimer>

// ============================================================
// TELEMETRY PANEL
// ============================================================
TelemetryPanel::TelemetryPanel(QWidget *parent)
    : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    // --- Drone identity ---
    m_nameLabel = new QLabel("Drone: --");
    QFont nameFont = m_nameLabel->font();
    nameFont.setPointSize(nameFont.pointSize() + 2);
    nameFont.setBold(true);
    m_nameLabel->setFont(nameFont);
    layout->addWidget(m_nameLabel);

    // --- Simulated battery level ---
    auto *batteryBox = new QGroupBox("Battery");
    auto *batteryLayout = new QHBoxLayout(batteryBox);

    m_batteryBar = new QProgressBar();
    m_batteryBar->setRange(0, 100);
    m_batteryBar->setValue(m_batteryLevel);
    m_batteryBar->setFormat("%p%");
    m_batteryBar->setTextVisible(true);

    m_batteryLabel = new QLabel("100%");
    batteryLayout->addWidget(m_batteryBar, 1);
    batteryLayout->addWidget(m_batteryLabel);
    layout->addWidget(batteryBox);

    // Battery is simulated locally in the GUI. It starts at 100%
    // and decreases by 1% every 18 seconds. No ROS topic is required.
    m_batteryTimer = new QTimer(this);
    connect(m_batteryTimer, &QTimer::timeout,
            this, &TelemetryPanel::updateBatteryLevel);
    m_batteryTimer->start(18000);

    // --- Perimeter status ---
    auto *perimeterBox = new QGroupBox("Bear Perimeter Status");
    auto *perimeterLayout = new QHBoxLayout(perimeterBox);
    m_trafficLight = new TrafficLightWidget();
    perimeterLayout->addWidget(m_trafficLight);
    perimeterLayout->addStretch();
    layout->addWidget(perimeterBox);

    // --- Deterrent status ---
    auto *deterrentBox = new QGroupBox("Deterrent");
    auto *deterrentLayout = new QHBoxLayout(deterrentBox);
    m_deterrentIndicator = new DeterrentIndicatorWidget();
    m_deterrentLabel = new QLabel("Awaiting data");
    deterrentLayout->addWidget(m_deterrentIndicator);
    deterrentLayout->addWidget(m_deterrentLabel);
    deterrentLayout->addStretch();
    layout->addWidget(deterrentBox);

    layout->addStretch();
}

// ============================================================
// UPDATE SIMULATED BATTERY LEVEL
// ============================================================
void TelemetryPanel::updateBatteryLevel()
{
    if (m_batteryLevel <= 0) {
        m_batteryTimer->stop();
        return;
    }

    --m_batteryLevel;
    m_batteryBar->setValue(m_batteryLevel);
    m_batteryLabel->setText(QString("%1%").arg(m_batteryLevel));
}

// ============================================================
// SET DRONE NAME
// ============================================================
void TelemetryPanel::setDroneName(const QString &name)
{
    m_nameLabel->setText(QString("Drone: %1").arg(name));
}

// ============================================================
// SET PERIMETER DISTANCE
// ============================================================
void TelemetryPanel::setPerimeterDistance(double distanceMeters)
{
    m_trafficLight->setDistanceToPerimeter(distanceMeters);
}

// ============================================================
// SET PERIMETER STATUS
// ============================================================
void TelemetryPanel::setPerimeterStatus(int status)
{
    switch (status) {
    case 1: m_trafficLight->setLevel(TrafficLightWidget::Level::Safe); break;
    case 2: m_trafficLight->setLevel(TrafficLightWidget::Level::Caution); break;
    case 3: m_trafficLight->setLevel(TrafficLightWidget::Level::Danger); break;
    default: m_trafficLight->setLevel(TrafficLightWidget::Level::Unknown); break;
    }
}

// ============================================================
// SET DETERRENT DEPLOYED
// ============================================================
void TelemetryPanel::setDeterrentDeployed(bool deployed)
{
    m_deterrentIndicator->setDeployed(deployed);
    m_deterrentLabel->setText(deployed ? "Deployed" : "Not deployed");
}

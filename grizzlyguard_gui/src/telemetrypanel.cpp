#include "telemetrypanel.h"
#include "trafficlightwidget.h"
#include "deterrentindicatorwidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QGroupBox>
#include <QFont>

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

    // --- Battery ---
    auto *batteryBox = new QGroupBox("Battery");
    auto *batteryLayout = new QHBoxLayout(batteryBox);
    m_batteryBar = new QProgressBar();
    m_batteryBar->setRange(0, 100);
    m_batteryLabel = new QLabel("--%");
    batteryLayout->addWidget(m_batteryBar);
    batteryLayout->addWidget(m_batteryLabel);
    layout->addWidget(batteryBox);

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
    m_deterrentLabel = new QLabel("Not deployed");
    deterrentLayout->addWidget(m_deterrentIndicator);
    deterrentLayout->addWidget(m_deterrentLabel);
    deterrentLayout->addStretch();
    layout->addWidget(deterrentBox);

    layout->addStretch();
}

void TelemetryPanel::setDroneName(const QString &name)
{
    m_nameLabel->setText(QString("Drone: %1").arg(name));
}

void TelemetryPanel::setBatteryPercent(int percent)
{
    m_batteryBar->setValue(percent);
    m_batteryLabel->setText(QString("%1%").arg(percent));

    // Color cue for low battery, independent of the bear traffic light.
    QString color = percent > 30 ? "green" : (percent > 15 ? "orange" : "red");
    m_batteryBar->setStyleSheet(QString("QProgressBar::chunk { background-color: %1; }").arg(color));
}

void TelemetryPanel::setPerimeterDistance(double distanceMeters)
{
    m_trafficLight->setDistanceToPerimeter(distanceMeters);
}

void TelemetryPanel::setDeterrentDeployed(bool deployed)
{
    m_deterrentIndicator->setDeployed(deployed);
    m_deterrentLabel->setText(deployed ? "Deployed" : "Not deployed");
}
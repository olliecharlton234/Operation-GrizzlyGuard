#include "perimeteralertdialog.h"

#include <QApplication>
#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

PerimeterAlertDialog::PerimeterAlertDialog(QWidget *parent)
    : QFrame(parent)
{
    setObjectName("alertBox");
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet(
        "#alertBox { background-color: #8e0000; border: 4px solid white; border-radius: 16px; }"
        "#alertBox QLabel { color: white; background: transparent; }"
        "#alertBox QPushButton { font-size: 18px; font-weight: bold; padding: 14px 28px; border-radius: 8px; }"
        "#alertBox QPushButton#ackButton { background-color: white; color: #8e0000; }");

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(40, 30, 40, 30);
    layout->setSpacing(20);

    auto *title = new QLabel("PERIMETER BREACH");
    QFont titleFont = title->font();
    titleFont.setPointSize(34);
    titleFont.setBold(true);
    title->setFont(titleFont);
    title->setAlignment(Qt::AlignCenter);
    layout->addWidget(title);

    m_detailLabel = new QLabel();
    QFont detailFont = m_detailLabel->font();
    detailFont.setPointSize(18);
    m_detailLabel->setFont(detailFont);
    m_detailLabel->setAlignment(Qt::AlignCenter);
    m_detailLabel->setWordWrap(true);
    layout->addWidget(m_detailLabel, 1);

    auto *buttonRow = new QHBoxLayout();
    buttonRow->setSpacing(20);
    buttonRow->addStretch();

    auto *ackButton = new QPushButton("Acknowledge");
    ackButton->setObjectName("ackButton");
    connect(ackButton, &QPushButton::clicked, this, &QWidget::hide);
    buttonRow->addWidget(ackButton);

    buttonRow->addStretch();
    layout->addLayout(buttonRow);

    hide(); // only shown on a breach

    // Re-centre whenever the main window is resized.
    if (parent)
        parent->installEventFilter(this);
}

bool PerimeterAlertDialog::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == parentWidget() && event->type() == QEvent::Resize && isVisible())
        centerInParent();
    return QFrame::eventFilter(watched, event);
}

void PerimeterAlertDialog::centerInParent()
{
    QWidget *p = parentWidget();
    if (!p)
        return;

    // About 60% of the window's width and 50% of its height, but never
    // smaller than a readable minimum and never bigger than the window.
    const int w = qMin(qMax(720, int(p->width() * 0.6)), p->width() - 40);
    const int h = qMin(qMax(400, int(p->height() * 0.5)), p->height() - 40);
    resize(w, h);
    move((p->width() - w) / 2, (p->height() - h) / 2);
}

void PerimeterAlertDialog::updateDistance(double metersOutside)
{
    m_detailLabel->setText(QString("A bear has left the perimeter.\nIt is currently %1 m outside.")
                               .arg(metersOutside, 0, 'f', 0));
}

void PerimeterAlertDialog::showBreach(double metersOutside)
{
    updateDistance(metersOutside);
    QApplication::beep();
    centerInParent();
    show();
    raise(); // keep it above the tabs
}
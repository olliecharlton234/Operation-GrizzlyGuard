// ============================================================
// DETERRENTINDICATORWIDGET.H
// Public interface and member declarations
// ============================================================

#ifndef DETERRENTINDICATORWIDGET_H
#define DETERRENTINDICATORWIDGET_H

#include <QWidget>

// A single lamp showing the bear deterrent state.
//   Grey with "?"   = no /deterrent/status message received yet
//   Dim amber ring  = received, not deployed
//   Bright orange   = deployed
class DeterrentIndicatorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DeterrentIndicatorWidget(QWidget *parent = nullptr);

    QSize sizeHint() const override { return QSize(80, 80); }
    QSize minimumSizeHint() const override { return QSize(60, 60); }

public slots:
    void setDeployed(bool deployed);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    bool m_hasData = false;   // false until the first status message arrives
    bool m_deployed = false;
};

#endif // DETERRENTINDICATORWIDGET_H
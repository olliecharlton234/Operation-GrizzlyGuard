#ifndef DETERRENTINDICATORWIDGET_H
#define DETERRENTINDICATORWIDGET_H


#include <QWidget>

// A single lamp showing whether the bear deterrent has been deployed.
// Gray/off = not deployed, orange/lit = deployed.
class DeterrentIndicatorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DeterrentIndicatorWidget(QWidget *parent = nullptr);

public slots:
    void setDeployed(bool deployed);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    bool m_deployed = false;
};

#endif // DETERRENTINDICATORWIDGET_H

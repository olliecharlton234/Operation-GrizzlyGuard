// ============================================================
// DETERRENTINDICATORWIDGET.H
// Public interface and member declarations
// ============================================================

#ifndef DETERRENTINDICATORWIDGET_H
#define DETERRENTINDICATORWIDGET_H


#include <QWidget>

// A single lamp showing whether the bear deterrent has been deployed.
// Gray/off = not deployed, orange/lit = deployed.
class DeterrentIndicatorWidget : public QWidget
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit DeterrentIndicatorWidget(QWidget *parent = nullptr);

    // -------------------------------------------------------
    // PUBLIC SLOTS
    // -------------------------------------------------------
public slots:
    void setDeployed(bool deployed);

    // -------------------------------------------------------
    // PROTECTED
    // -------------------------------------------------------
protected:
    void paintEvent(QPaintEvent *event) override;

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    bool m_deployed = false; // Lamp off until real status arrives.
};

#endif // DETERRENTINDICATORWIDGET_H

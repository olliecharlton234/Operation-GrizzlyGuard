// ============================================================
// PERIMETERALERTDIALOG.H
// Public interface and member declarations
// ============================================================

#ifndef PERIMETERALERTDIALOG_H
#define PERIMETERALERTDIALOG_H

#include <QFrame>

class QLabel;

// Large red emergency box shown when a bear leaves the perimeter.
//
// It is drawn INSIDE the main window (as a child of it), centred and sized
// relative to the window, so it sits in the middle of whichever tab is open
// and stays centred when the window is resized. (The class name is kept from
// the earlier pop-up version so no other file needs renaming.)
//
// Pass the main window as the parent.
class PerimeterAlertDialog : public QFrame
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit PerimeterAlertDialog(QWidget *parent = nullptr);

    // Shows the box (or brings it back to the front) with the distance, in
    // meters, that the bear is outside the perimeter.
    void showBreach(double metersOutside);

    // Refreshes the distance text while the box is already open.
    void updateDistance(double metersOutside);

    // -------------------------------------------------------
    // PROTECTED
    // -------------------------------------------------------
protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    void centerInParent();

    QLabel *m_detailLabel;
};




#endif // PERIMETERALERTDIALOG_H

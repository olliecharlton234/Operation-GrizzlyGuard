// ============================================================
// VIDEOSTREAMRECEIVER.H
// Public interface and member declarations
// ============================================================

#ifndef VIDEOSTREAMRECEIVER_H
#define VIDEOSTREAMRECEIVER_H

#include <QObject>
#include <QImage>
#include <QByteArray>

class QTcpSocket;
class QTimer;

class VideoStreamReceiver : public QObject
{
    Q_OBJECT

    // -------------------------------------------------------
    // PUBLIC
    // -------------------------------------------------------
public:
    explicit VideoStreamReceiver(QObject *parent = nullptr);

    void start(); // begins trying to connect
    bool isConnected() const;

    // -------------------------------------------------------
    // SIGNALS
    // -------------------------------------------------------
signals:
    void frameReady(const QImage &frame);
    void connectionStatusChanged(bool connected);

    // -------------------------------------------------------
    // PRIVATE SLOTS
    // -------------------------------------------------------
private slots:
    void attemptConnect();
    void onSocketConnected();
    void onSocketDisconnected();
    void onSocketReadyRead();
    void onSocketError();

    // -------------------------------------------------------
    // PRIVATE
    // -------------------------------------------------------
private:
    QTcpSocket *m_socket;
    QTimer *m_retryTimer;
    QByteArray m_recvBuffer;
    quint32 m_expectedFrameSize = 0; // 0 = still waiting for the 4-byte length header
};
#endif // VIDEOSTREAMRECEIVER_H

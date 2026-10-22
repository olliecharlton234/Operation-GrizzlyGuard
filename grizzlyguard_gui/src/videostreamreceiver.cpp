// ============================================================
// VIDEOSTREAMRECEIVER.CPP
// Qt / ROS 2 GUI implementation
// ============================================================

#include "videostreamreceiver.h"

#include <QTcpSocket>
#include <QTimer>
#include <QDataStream>
#include <QDebug>

namespace {
constexpr const char *kVideoHost = "127.0.0.1";
constexpr quint16 kVideoPort = 5556; // separate port from any telemetry socket
constexpr int kRetryIntervalMs = 2000;
}

// ============================================================
// VIDEO STREAM RECEIVER
// ============================================================
VideoStreamReceiver::VideoStreamReceiver(QObject *parent)
    : QObject(parent)
{
    m_socket = new QTcpSocket(this);
    connect(m_socket, &QTcpSocket::connected, this, &VideoStreamReceiver::onSocketConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &VideoStreamReceiver::onSocketDisconnected);
    connect(m_socket, &QTcpSocket::readyRead, this, &VideoStreamReceiver::onSocketReadyRead);
    connect(m_socket, &QTcpSocket::errorOccurred, this, &VideoStreamReceiver::onSocketError);

    m_retryTimer = new QTimer(this);
    connect(m_retryTimer, &QTimer::timeout, this, &VideoStreamReceiver::attemptConnect);
}

// ============================================================
// START
// ============================================================
void VideoStreamReceiver::start()
{
    attemptConnect();
    m_retryTimer->start(kRetryIntervalMs);
}

// ============================================================
// IS CONNECTED
// ============================================================
bool VideoStreamReceiver::isConnected() const
{
    return m_socket->state() == QAbstractSocket::ConnectedState;
}

// ============================================================
// ATTEMPT CONNECT
// ============================================================
void VideoStreamReceiver::attemptConnect()
{
    if (isConnected() || m_socket->state() == QAbstractSocket::ConnectingState)
        return;
    m_socket->connectToHost(kVideoHost, kVideoPort);
}

// ============================================================
// ON SOCKET CONNECTED
// ============================================================
void VideoStreamReceiver::onSocketConnected()
{
    qDebug() << "[VideoStreamReceiver] Connected to video source";
    m_recvBuffer.clear();
    m_expectedFrameSize = 0;
    emit connectionStatusChanged(true);
}

// ============================================================
// ON SOCKET DISCONNECTED
// ============================================================
void VideoStreamReceiver::onSocketDisconnected()
{
    qDebug() << "[VideoStreamReceiver] Video source disconnected - will keep retrying";
    emit connectionStatusChanged(false);
}

// ============================================================
// ON SOCKET ERROR
// ============================================================
void VideoStreamReceiver::onSocketError()
{
    // Includes "connection refused" while the engine isn't up yet - expected, not fatal.
    qDebug() << "[VideoStreamReceiver] Socket error (will keep retrying):" << m_socket->errorString();
}

// ============================================================
// ON SOCKET READY READ
// ============================================================
void VideoStreamReceiver::onSocketReadyRead()
{
    m_recvBuffer += m_socket->readAll();

    // Process as many complete [length][data] frames as are currently buffered.
    while (true) {
        if (m_expectedFrameSize == 0) {
            // Waiting for the 4-byte length header.
            if (m_recvBuffer.size() < 4)
                return; // not enough data yet - wait for more

            QDataStream stream(m_recvBuffer.left(4));
            stream.setByteOrder(QDataStream::LittleEndian);
            stream >> m_expectedFrameSize;
            m_recvBuffer.remove(0, 4);

            // Sanity check - reject absurd sizes rather than trying to
            // buffer gigabytes if the protocol gets out of sync somehow.
            if (m_expectedFrameSize == 0 || m_expectedFrameSize > 50 * 1024 * 1024) {
                qWarning() << "[VideoStreamReceiver] Bad frame length, dropping connection:"
                           << m_expectedFrameSize;
                m_socket->disconnectFromHost();
                m_expectedFrameSize = 0;
                m_recvBuffer.clear();
                return;
            }
        }

        if (static_cast<quint32>(m_recvBuffer.size()) < m_expectedFrameSize)
            return; // full frame not buffered yet - wait for more data

        QByteArray jpegData = m_recvBuffer.left(m_expectedFrameSize);
        m_recvBuffer.remove(0, m_expectedFrameSize);
        m_expectedFrameSize = 0; // ready to read the next frame's length header

        QImage frame;
        if (frame.loadFromData(jpegData, "JPG")) {
            emit frameReady(frame);
        } else {
            qWarning() << "[VideoStreamReceiver] Failed to decode frame data";
        }
    }
}

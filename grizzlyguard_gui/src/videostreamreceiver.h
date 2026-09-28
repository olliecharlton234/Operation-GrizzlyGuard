#ifndef VIDEOSTREAMRECEIVER_H
#define VIDEOSTREAMRECEIVER_H

#include <QObject>
#include <QImage>
#include <QByteArray>

class QTcpSocket;
class QTimer;

// ============================================================================
// Receives live camera frames from an external process (Unity/Unreal/Gazebo/
// etc.) over a TCP socket - this is separate from SimulationWorker because
// a game engine runs as its OWN application, not as a thread inside this
// GUI's process, so frames have to cross a real process boundary.
//
// PROTOCOL: on the socket, each frame is sent as:
//   [4-byte little-endian uint32: length N] [N bytes: JPEG-encoded frame]
// repeated for every frame, with no other framing. This is about as simple
// a protocol as exists, so it's realistic for whichever engine ends up
// being used to implement on their side:
//   - Gazebo: a small bridge node subscribing to the camera topic
//     (sensor_msgs/Image via ROS, or the engine's native image topic),
//     re-encoding each frame to JPEG, and writing it length-prefixed to
//     a TCP socket.
//   - Unity: a C# script capturing the render texture each frame,
//     Texture2D.EncodeToJPG(), and writing it length-prefixed to a
//     System.Net.Sockets.TcpClient/NetworkStream.
//   - Unreal: similar idea via a capture component + FImageUtils, written
//     out over a raw socket.
//
// Connects to 127.0.0.1:kVideoPort by default (change in the .cpp) and
// retries every 2 seconds if nothing is listening yet - same "wait for the
// simulation to start" behavior as the rest of the app.
// ============================================================================
class VideoStreamReceiver : public QObject
{
    Q_OBJECT

public:
    explicit VideoStreamReceiver(QObject *parent = nullptr);

    void start(); // begins trying to connect
    bool isConnected() const;

signals:
    void frameReady(const QImage &frame);
    void connectionStatusChanged(bool connected);

private slots:
    void attemptConnect();
    void onSocketConnected();
    void onSocketDisconnected();
    void onSocketReadyRead();
    void onSocketError();

private:
    QTcpSocket *m_socket;
    QTimer *m_retryTimer;
    QByteArray m_recvBuffer;
    quint32 m_expectedFrameSize = 0; // 0 = still waiting for the 4-byte length header
};
#endif // VIDEOSTREAMRECEIVER_H

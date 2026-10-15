package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Queue;

/**
 * Receives MPU-6050 yaw and angular velocity from a Raspberry Pi over UDP.
 *
 * <p>The Raspberry Pi sends two big-endian doubles:
 *
 * <ul>
 *   <li>Yaw in radians
 *   <li>Yaw angular velocity in radians per second
 * </ul>
 */
public class GyroIOMPU6050 implements GyroIO {
  private static final int PACKET_SIZE_BYTES = Double.BYTES * 2;
  private static final long CONNECTION_TIMEOUT_NS = 250_000_000L;

  private final DatagramSocket socket;
  private final Thread receiverThread;

  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> yawPositionQueue;

  private volatile double latestYawRad = 0.0;
  private volatile double latestYawRateRadPerSec = 0.0;
  private volatile long lastPacketTimeNs = 0L;

  public GyroIOMPU6050(int port) {
    try {
      socket = new DatagramSocket(port);
      socket.setSoTimeout(100);
    } catch (SocketException exception) {
      throw new RuntimeException("Could not open MPU-6050 UDP port " + port, exception);
    }

    /*
     * The odometry thread samples the most recently received Pi yaw at
     * exactly the same time as the Spark encoder signals.
     */
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> latestYawRad);

    receiverThread = new Thread(this::receiveLoop, "MPU6050-UDP-Receiver");
    receiverThread.setDaemon(true);
    receiverThread.start();
  }

  private void receiveLoop() {
    byte[] buffer = new byte[PACKET_SIZE_BYTES];
    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

    while (!Thread.currentThread().isInterrupted()) {
      try {
        // Reset the maximum packet length before receiving again.
        packet.setLength(buffer.length);
        socket.receive(packet);

        if (packet.getLength() != PACKET_SIZE_BYTES) {
          continue;
        }

        ByteBuffer data =
            ByteBuffer.wrap(packet.getData(), packet.getOffset(), packet.getLength())
                .order(ByteOrder.BIG_ENDIAN);

        double receivedYawRad = data.getDouble();
        double receivedRateRadPerSec = data.getDouble();

        if (!Double.isFinite(receivedYawRad) || !Double.isFinite(receivedRateRadPerSec)) {
          continue;
        }

        latestYawRad = receivedYawRad;
        latestYawRateRadPerSec = receivedRateRadPerSec;

        // Write this last so the other values are already available.
        lastPacketTimeNs = System.nanoTime();

      } catch (SocketTimeoutException ignored) {
        // Allows the loop to periodically check for interruption.
      } catch (IOException exception) {
        if (!socket.isClosed()) {
          exception.printStackTrace();
        }
      }
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    long packetAgeNs = System.nanoTime() - lastPacketTimeNs;

    inputs.connected = lastPacketTimeNs != 0L && packetAgeNs < CONNECTION_TIMEOUT_NS;

    inputs.yawPosition = Rotation2d.fromRadians(latestYawRad);
    inputs.yawVelocityRadPerSec = latestYawRateRadPerSec;

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}

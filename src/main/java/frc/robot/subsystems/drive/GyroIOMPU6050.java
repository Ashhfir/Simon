package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;

/**
 * IO implementation for an MPU-6050 connected directly to the roboRIO MXP I2C bus.
 *
 * <p>Mount the sensor flat and rigidly with its +Z axis pointing up. It measures yaw by integrating
 * Z-axis angular velocity, so some long-term drift is expected. Keep the robot stationary for the
 * first few seconds after startup so the gyro bias can be calibrated.
 */
public class GyroIOMPU6050 implements GyroIO {
  private static final int i2cAddress = 0x68; // Use 0x69 if AD0 is tied high
  private static final int whoAmIRegister = 0x75;
  private static final int powerManagementRegister = 0x6B;
  private static final int sampleRateRegister = 0x19;
  private static final int filterRegister = 0x1A;
  private static final int gyroConfigRegister = 0x1B;
  private static final int gyroZRegister = 0x47;

  // +/-1000 deg/s = 32.8 counts per deg/s (avoids clipping at normal swerve turn speeds).
  private static final double countsPerDegreePerSecond = 32.8;
  private static final int calibrationSamplesRequired = 200; // ~2 seconds at 100 Hz
  private static final long settleTimeMicros = 250_000; // Allow the gyro to stabilize after wakeup
  private static final long connectionTimeoutMicros = 250_000;

  private final I2C mpu = new I2C(I2C.Port.kMXP, i2cAddress);
  private final byte[] gyroBytes = new byte[2];
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> yawPositionQueue;

  // Accessed only by the odometry thread and updateInputs(), under Drive.odometryLock.
  private final long calibrationStartMicros = RobotController.getFPGATime() + settleTimeMicros;
  private boolean initialized = false;
  private boolean calibrated = false;
  private int calibrationSamples = 0;
  private double calibrationSum = 0.0;
  private double gyroBiasCounts = 0.0;
  private double yawRad = 0.0;
  private double yawRateRadPerSec = 0.0;
  private long previousSampleMicros = 0;
  private long lastGoodReadMicros = 0;

  public GyroIOMPU6050() {
    byte[] id = new byte[1];
    boolean readAborted = mpu.read(whoAmIRegister, id.length, id);

    if (readAborted) {
      DriverStation.reportError(
          "[MPU6050] WHO_AM_I read ABORTED on MXP I2C at 0x68. Check power, ground, SDA, SCL, and AD0.",
          false);
    } else {
      DriverStation.reportWarning(
          String.format("[MPU6050] WHO_AM_I returned 0x%02X (expected 0x68)", id[0] & 0xFF),
          false);
    }

    if (!readAborted && (id[0] & 0xFF) == 0x68) {
      // Keep the same register settings; check each write independently for diagnostics.
      boolean wakeOk = !mpu.write(powerManagementRegister, 0x01);
      boolean filterOk = !mpu.write(filterRegister, 0x03);
      boolean rangeOk = !mpu.write(gyroConfigRegister, 0x10);
      boolean rateOk = !mpu.write(sampleRateRegister, 0x09);

      initialized = wakeOk && filterOk && rangeOk && rateOk;
      DriverStation.reportWarning(
          String.format(
              "[MPU6050] Config writes: wake=%s filter=%s gyroRange=%s sampleRate=%s",
              wakeOk, filterOk, rangeOk, rateOk),
          false);
    }

    if (!initialized) {
      DriverStation.reportError("[MPU6050] Initialization FAILED; yaw will be unavailable.", false);
    } else {
      DriverStation.reportWarning(
          "[MPU6050] Initialization OK; keep robot still during gyro calibration.", false);
    }

    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(this::sampleYaw);
  }

  /** Called by SparkOdometryThread at the same frequency as the module encoders. */
  private double sampleYaw() {
    if (!initialized) {
      return yawRad;
    }

    long nowMicros = RobotController.getFPGATime();
    if (mpu.read(gyroZRegister, gyroBytes.length, gyroBytes)) {
      // Skip integrating an unknown interval when a read fails.
      previousSampleMicros = 0;
      return yawRad;
    }
    lastGoodReadMicros = nowMicros;

    // The MPU-6050 stores each axis as a signed, big-endian 16-bit integer.
    short rawZ = (short) (((gyroBytes[0] & 0xFF) << 8) | (gyroBytes[1] & 0xFF));

    if (!calibrated) {
      if (nowMicros >= calibrationStartMicros) {
        calibrationSum += rawZ;
        calibrationSamples++;
        if (calibrationSamples >= calibrationSamplesRequired) {
          gyroBiasCounts = calibrationSum / calibrationSamples;
          calibrated = true;
          previousSampleMicros = nowMicros;
          DriverStation.reportWarning("[MPU6050] Gyro calibration complete.", false);
        }
      }
      return yawRad;
    }

    // Positive Z points up; positive yaw is counterclockwise, matching WPILib.
    yawRateRadPerSec = Math.toRadians((rawZ - gyroBiasCounts) / countsPerDegreePerSecond);
    if (previousSampleMicros != 0) {
      double dt = (nowMicros - previousSampleMicros) / 1e6;
      if (dt > 0.0 && dt < 0.1) {
        yawRad += yawRateRadPerSec * dt;
      }
    }
    previousSampleMicros = nowMicros;
    return yawRad;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        calibrated
            && lastGoodReadMicros != 0
            && RobotController.getFPGATime() - lastGoodReadMicros < connectionTimeoutMicros;
    inputs.yawPosition = Rotation2d.fromRadians(yawRad);
    inputs.yawVelocityRadPerSec = yawRateRadPerSec;

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}

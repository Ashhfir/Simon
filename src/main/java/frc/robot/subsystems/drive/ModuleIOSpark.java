package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for a Falcon 500/TalonFX drive motor and a NEO/Spark Max turn motor.
 *
 * <p>The SRX Mag Encoder is used in the same high-level arrangement as the 2024 robot:
 *
 * <ul>
 *   <li>A/B quadrature -> Spark Max alternate encoder, used for normal steering feedback/odometry.
 *   <li>Absolute PWM -> roboRIO DIO, used to seed the quadrature position at startup.
 * </ul>
 *
 * <p>This keeps the AdvantageKit-style IO structure while using the sensor arrangement already
 * built into the modules.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Drive hardware: Falcon 500 with its built-in TalonFX controller and encoder.
  private final TalonFX driveTalon;
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Drive control requests.
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0);

  // Turn hardware: NEO/Spark Max, with SRX Mag quadrature feedback through the Spark data port.
  private final SparkMax turnSpark;
  private final RelativeEncoder turnEncoder;
  private final DutyCycleEncoder turnAbsoluteEncoder;
  private final SparkClosedLoopController turnController;
  private boolean turnEncoderReady = false;
  private boolean turnEncoderSynced = false;

  // Queue inputs from the Spark odometry thread.
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers.
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOSpark(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> throw new IllegalArgumentException("Invalid module index: " + module);
        };

    int driveCanId =
        switch (module) {
          case 0 -> frontLeftDriveCanId;
          case 1 -> frontRightDriveCanId;
          case 2 -> backLeftDriveCanId;
          case 3 -> backRightDriveCanId;
          default -> throw new IllegalArgumentException("Invalid module index: " + module);
        };

    int turnCanId =
        switch (module) {
          case 0 -> frontLeftTurnCanId;
          case 1 -> frontRightTurnCanId;
          case 2 -> backLeftTurnCanId;
          case 3 -> backRightTurnCanId;
          default -> throw new IllegalArgumentException("Invalid module index: " + module);
        };

    boolean driveInverted =
        switch (module) {
          case 0 -> frontLeftDriveInverted;
          case 1 -> frontRightDriveInverted;
          case 2 -> backLeftDriveInverted;
          case 3 -> backRightDriveInverted;
          default -> throw new IllegalArgumentException("Invalid module index: " + module);
        };

    int turnEncoderDio =
        switch (module) {
          case 0 -> frontLeftTurnEncoderDio;
          case 1 -> frontRightTurnEncoderDio;
          case 2 -> backLeftTurnEncoderDio;
          case 3 -> backRightTurnEncoderDio;
          default -> throw new IllegalArgumentException("Invalid module index: " + module);
        };

    driveTalon = new TalonFX(driveCanId);
    turnSpark = new SparkMax(turnCanId, MotorType.kBrushless);

    // Absolute SRX Mag PWM signal on roboRIO DIO. This is only needed to establish the module's
    // position at startup; steering feedback after that comes from A/B quadrature.
    turnAbsoluteEncoder = new DutyCycleEncoder(turnEncoderDio);
    turnAbsoluteEncoder.setInverted(turnAbsoluteEncoderInverted);
    turnAbsoluteEncoder.setConnectedFrequencyThreshold(200);

    // SRX Mag A/B quadrature through the Spark Max data-port ribbon cable.
    turnEncoder = turnSpark.getAlternateEncoder();
    turnController = turnSpark.getClosedLoopController();

    // Configure Falcon drive motor and its built-in encoder.
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.Feedback.SensorToMechanismRatio = driveMotorReduction;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // These gains are Phoenix voltage-control gains. Tune them for the TalonFX;
    // do not assume Spark PID values will transfer directly.
    driveConfig.Slot0.kP = driveKp;
    driveConfig.Slot0.kD = driveKd;

    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure the SRX Mag A/B quadrature input as the Spark Max alternate encoder.
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .alternateEncoder
        .setSparkMaxDataPortConfig()
        .countsPerRevolution(turnEncoderCountsPerRevolution)
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .measurementPeriod(10)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        .externalOrAltEncoderPositionAlwaysOn(true)
        .externalOrAltEncoderPosition((int) (1000.0 / odometryFrequency))
        .externalOrAltEncoderVelocityAlwaysOn(true)
        .externalOrAltEncoderVelocity(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create TalonFX status signals.
    // Because SensorToMechanismRatio is set, position and velocity are wheel rotations and wheel
    // RPS.
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequency, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon);

    // Keep SparkOdometryThread because the turn motor/encoder are on the Spark Max.
    // Its generic registerSignal overload also samples the TalonFX drive position in the same
    // timestamped loop.
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(() -> drivePosition.getValueAsDouble());
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update Falcon drive inputs.
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Seed the relative/quadrature steering encoder from the absolute PWM encoder once while
    // disabled. Do not invent a zero if the absolute signal is missing; leave the module inhibited
    // so a wiring problem is obvious instead of silently creating an incorrect steering offset.
    inputs.turnAbsoluteEncoderConnected = turnAbsoluteEncoder.isConnected();
    if (DriverStation.isDisabled() && !turnEncoderSynced && inputs.turnAbsoluteEncoderConnected) {
      double absoluteRadians =
          MathUtil.inputModulus(
              Rotation2d.fromRotations(turnAbsoluteEncoder.get()).getRadians(),
              turnPIDMinInput,
              turnPIDMaxInput);

      if (turnEncoder.setPosition(absoluteRadians) == REVLibError.kOk) {
        turnEncoderReady = true;
        turnEncoderSynced = true;
      }
    }
    inputs.turnEncoderReady = turnEncoderReady;
    inputs.turnEncoderSynced = turnEncoderSynced;

    // Update Spark Max turn inputs.
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs.
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(driveVoltageRequest.withOutput(turnEncoderReady ? output : 0.0));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(turnEncoderReady ? output : 0.0);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    if (!turnEncoderReady) {
      driveTalon.setControl(driveVoltageRequest.withOutput(0.0));
      return;
    }

    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;

    driveTalon.setControl(
        driveVelocityRequest.withVelocity(velocityRotPerSec).withFeedForward(ffVolts));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    if (!turnEncoderReady) {
      turnSpark.setVoltage(0.0);
      return;
    }

    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }
}

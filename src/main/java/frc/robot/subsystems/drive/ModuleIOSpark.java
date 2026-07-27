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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
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
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for a Falcon 500/TalonFX drive motor, a NEO/Spark Max turn motor, and a
 * duty-cycle absolute encoder connected to the turn Spark Max.
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

  // Turn hardware: NEO controlled by a Spark Max.
  private final SparkBase turnSpark;
  private final AbsoluteEncoder turnEncoder;
  private final SparkClosedLoopController turnController;

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
          default -> Rotation2d.kZero;
        };

    int driveCanId =
        switch (module) {
          case 0 -> frontLeftDriveCanId;
          case 1 -> frontRightDriveCanId;
          case 2 -> backLeftDriveCanId;
          case 3 -> backRightDriveCanId;
          default -> 0;
        };

    int turnCanId =
        switch (module) {
          case 0 -> frontLeftTurnCanId;
          case 1 -> frontRightTurnCanId;
          case 2 -> backLeftTurnCanId;
          case 3 -> backRightTurnCanId;
          default -> 0;
        };

    driveTalon = new TalonFX(driveCanId);
    turnSpark = new SparkMax(turnCanId, MotorType.kBrushless);
    turnEncoder = turnSpark.getAbsoluteEncoder();
    turnController = turnSpark.getClosedLoopController();

    // Configure Falcon drive motor and its built-in encoder.
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = driveMotorReduction;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // These gains are Phoenix voltage-control gains. Tune them for the TalonFX;
    // do not assume Spark PID values will transfer directly.
    driveConfig.Slot0.kP = driveKp;
    driveConfig.Slot0.kD = driveKd;

    // Add MotorOutput.Inverted here if the Falcon direction is reversed.
    // Example:
    // driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure Spark Max turn motor and duty-cycle absolute encoder.
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
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

    // Keep SparkOdometryThread because the turn motor is a Spark.
    // Its generic registerSignal overload allows the TalonFX signal to be sampled with the Spark.
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
    driveTalon.setControl(driveVoltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;

    driveTalon.setControl(
        driveVelocityRequest.withVelocity(velocityRotPerSec).withFeedForward(ffVolts));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }
}

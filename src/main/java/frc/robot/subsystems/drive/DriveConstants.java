// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(26.5);
  public static final double wheelBase = Units.inchesToMeters(26.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-0.1104465802405206);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(1.2363885561730537);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(-1.558523130401064);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(-0.7317084789118062);

  // Device CAN IDs
  // Reserved for the optional Pigeon implementation; no gyro is constructed in RobotContainer.
  public static final int pigeonCanId = 9;

  public static final int frontLeftDriveCanId = 1;
  public static final int frontRightDriveCanId = 3;
  public static final int backLeftDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  // Per-module drive inversion, verified with all modules commanded straight forward.
  public static final boolean frontLeftDriveInverted = false;
  public static final boolean frontRightDriveInverted = true;
  public static final boolean backLeftDriveInverted = false;
  public static final boolean backRightDriveInverted = true;

  public static final int frontLeftTurnCanId = 2;
  public static final int frontRightTurnCanId = 4;
  public static final int backLeftTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  // SRX Mag Encoder PWM outputs connected to roboRIO DIO
  public static final int frontLeftTurnEncoderDio = 0;
  public static final int frontRightTurnEncoderDio = 1;
  public static final int backLeftTurnEncoderDio = 2;
  public static final int backRightTurnEncoderDio = 3;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final double driveMotorReduction = 6.75; // SDS MK4i L2
  public static final DCMotor driveGearbox = DCMotor.getFalcon500(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 150.0 / 7.0; // SDS MK4i
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  // SRX Mag A/B quadrature is connected to the Spark Max alternate-encoder data port.
  // One encoder revolution is one full module revolution (no steering gear ratio here).
  public static final int turnEncoderCountsPerRevolution = 4096;
  public static final boolean turnEncoderInverted = false;
  public static final boolean turnAbsoluteEncoderInverted = false;
  public static final double turnEncoderPositionFactor = 2.0 * Math.PI;
  public static final double turnEncoderVelocityFactor = 2.0 * Math.PI / 60.0;

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}

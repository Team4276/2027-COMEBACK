package frc.team4276.frc2026.subsystems.drive;

import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import frc.team4276.frc2026.Ports;

public class DriveConstants {
  public static final int odometryFrequency = 200;
  public static final double trackWidth = Units.inchesToMeters(19.5);
  public static final double wheelBase = Units.inchesToMeters(27.5);
  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
  };

  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

  public static final double maxVelocityMPS = 4.92;
  public static final double maxAngularVelocity = 10.54;

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(1.52507);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0318);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.034);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(2.5579);

  public static final Rotation2d frontLeftZeroHelperRotation = Rotation2d.kCCW_90deg;
  public static final Rotation2d frontRightZeroHelperRotation = Rotation2d.kZero;
  public static final Rotation2d backLeftZeroHelperRotation = Rotation2d.k180deg;
  public static final Rotation2d backRightZeroHelperRotation = Rotation2d.kCW_90deg;

  // Device CAN IDs
  public static final int frontLeftDriveCanId = Ports.FRONT_LEFT_DRIVE;
  public static final int frontRightDriveCanId = Ports.FRONT_RIGHT_DRIVE;
  public static final int backLeftDriveCanId = Ports.BACK_LEFT_DRIVE;
  public static final int backRightDriveCanId = Ports.BACK_RIGHT_DRIVE;

  public static final int frontLeftTurnCanId = Ports.FRONT_LEFT_TURN;
  public static final int frontRightTurnCanId = Ports.FRONT_RIGHT_TURN;
  public static final int backLeftTurnCanId = Ports.BACK_LEFT_TURN;
  public static final int backRightTurnCanId = Ports.BACK_RIGHT_TURN;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 40;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.47);
  public static final double drivingMotorPinionTeeth = 12.0;
  public static final double driveMotorReduction = (45.0 * 22.0) / (drivingMotorPinionTeeth * 15.0);

  public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);
  public static final double maxSteerVelocity = driveGearbox.freeSpeedRadPerSec / driveMotorReduction;

  public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;
  public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction;

  // Drive PID configuration
  public static final double driveKp = 0.006;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.03051;
  public static final double driveKv = 0.0901;
  public static final double driveKa = 0.004;
  public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);

  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.0;
  
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
}

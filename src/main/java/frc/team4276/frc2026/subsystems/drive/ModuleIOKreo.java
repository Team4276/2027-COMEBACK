package frc.team4276.frc2026.subsystems.drive;

import static frc.team4276.frc2026.subsystems.drive.DriveConstants.*;
import static frc.team4276.lib.SparkUtil.*;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;

public class ModuleIOKreo implements ModuleIO {
  private final Rotation2d zeroRotation;
  private final Rotation2d zeroHelperRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final SparkMax turnSpark;
  private final StatusSignal<Angle> drivePositionSignal;
  private final AbsoluteEncoder turnEncoder;
  private final SparkMaxConfig turnConfig;

  // Closed loop controllers
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  private boolean brakeModeEnabled = true;

  public ModuleIOKreo(int module) {
    zeroRotation = switch (module) {
      case 0 -> frontLeftZeroRotation;
      case 1 -> frontRightZeroRotation;
      case 2 -> backLeftZeroRotation;
      case 3 -> backRightZeroRotation;
      default -> Rotation2d.kZero;
    };
    zeroHelperRotation = switch (module) {
      case 0 -> frontLeftZeroHelperRotation;
      case 1 -> frontRightZeroHelperRotation;
      case 2 -> backLeftZeroHelperRotation;
      case 3 -> backRightZeroHelperRotation;
      default -> Rotation2d.kZero;
    };
    driveTalon = new TalonFX(
        switch (module) {
          case 0 -> frontLeftDriveCanId;
          case 1 -> frontRightDriveCanId;
          case 2 -> backLeftDriveCanId;
          case 3 -> backRightDriveCanId;
          default -> 0;
        });
    turnSpark = new SparkMax(
        switch (module) {
          case 0 -> frontLeftTurnCanId;
          case 1 -> frontRightTurnCanId;
          case 2 -> backLeftTurnCanId;
          case 3 -> backRightTurnCanId;
          default -> 0;
        },
        MotorType.kBrushless);
    drivePositionSignal = driveTalon.getPosition();
    turnEncoder = turnSpark.getAbsoluteEncoder();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    // driveConfig
    //     .idleMode(IdleMode.kBrake)
    //     .smartCurrentLimit(driveMotorCurrentLimit)
    //     .voltageCompensation(12.0);
    // driveConfig.encoder
    //     .positionConversionFactor(driveEncoderPositionFactor)
    //     .velocityConversionFactor(driveEncoderVelocityFactor)
    //     .uvwMeasurementPeriod(10)
    //     .uvwAverageDepth(2);
    // driveConfig.closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .pid(
    //         driveKp,
    //         0.0,
    //         driveKd);
    // driveConfig.signals
    //     .primaryEncoderPositionAlwaysOn(true)
    //     .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);

    // Configure turn motor
    turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig.absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2 * Math.PI)
        .pid(turnKp, 0.0, turnKd);
    turnConfig.signals
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
        () -> turnSpark.configure(
            turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePositionSignal);
    turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    // ifOk(driveSpark, drivePositionSignal::getPosition, (value) -> inputs.drivePositionRad = value);
    // ifOk(driveSpark, drivePositionSignal::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    // ifOk(
    //     driveSpark,
    //     new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
    //     (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    // ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.zeroHelperTurnPosition = inputs.turnPosition.minus(zeroHelperRotation);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
        .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
        .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(new VoltageOut(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadPerSec) {
    runDriveVelocitySetpoint(velocityRadPerSec, false);
  }

  private double lastVelocity = 0.0;

  @Override
  public void runDriveVelocitySetpoint(double velocityRadPerSec, boolean useAccel) {
    double ffVolts;
    if (useAccel) {
      ffVolts = feedforward.calculateWithVelocities(lastVelocity, velocityRadPerSec);

    } else {
      ffVolts = 0.0;
      // feedforward.calculate(velocityRadPerSec);
    }
    lastVelocity = velocityRadPerSec;

    // driveController.setSetpoint(
    //     velocityRadPerSec,
    //     ControlType.kVelocity,
    //     ClosedLoopSlot.kSlot0,
    //     ffVolts,
    //     ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint = MathUtil.inputModulus(
        rotation.plus(zeroRotation).getRadians(), 0, 2 * Math.PI);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled)
      return;
    brakeModeEnabled = enabled;
    // new Thread(
    //     () -> {
    //       tryUntilOk(
    //           driveSpark,
    //           5,
    //           () -> driveSpark.configure(
    //               driveConfig.idleMode(
    //                   brakeModeEnabled
    //                       ? SparkBaseConfig.IdleMode.kBrake
    //                       : SparkBaseConfig.IdleMode.kCoast),
    //               ResetMode.kNoResetSafeParameters,
    //               PersistMode.kNoPersistParameters));
    //     })
    //     .start();
  }
    
}

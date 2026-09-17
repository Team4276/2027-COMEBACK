package frc.team4276.frc2026.subsystems.drive;

import static frc.team4276.frc2026.subsystems.drive.DriveConstants.*;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.Models;
import org.wpilib.system.Timer;
import org.wpilib.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  // LinearSystemId was removed in 2027 - Models.singleJointedArmFromPhysicalConstants(...)
  // is the replacement (same (DCMotor, momentOfInertia, gearing) signature as the old
  // LinearSystemId.createDCMotorSystem(...) - a bare spinning motor is mathematically a
  // single-jointed arm without gravity).
  private final DCMotorSim driveSim =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(driveGearbox, 0.025, driveMotorReduction),
          driveGearbox);
  private final DCMotorSim turnSim =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(turnGearbox, 0.004, turnMotorReduction), turnGearbox);

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private PIDController turnController = new PIDController(turnSimP, 0, turnSimD);

  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocity());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPosition());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(Math.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(Math.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPosition();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDraw());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocity();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDraw());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadPerSec) {
    runDriveVelocitySetpoint(velocityRadPerSec, false);
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadPerSec, boolean useAccel) {
    driveClosedLoop = true;
    driveFFVolts =
        (driveSimKs * Math.signum(velocityRadPerSec))
            + (driveSimKv * velocityRadPerSec);
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}

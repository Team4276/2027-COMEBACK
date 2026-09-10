package frc.team4276.frc2026.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import frc.team4276.frc2026.RobotState;
import frc.team4276.frc2026.FieldConstants.FieldZone;
import frc.team4276.frc2026.shooter.ShotCalculator;
import frc.team4276.frc2026.shooter.ShooterConstants.ParamPreset;
import frc.team4276.frc2026.shooter.ShotCalculator.ShootingParameters;
import frc.team4276.frc2026.subsystems.drive.Drive;
import frc.team4276.frc2026.subsystems.drive.Drive.DriveSpeedScalar;
import frc.team4276.frc2026.subsystems.feeder.Feeder;
import frc.team4276.frc2026.subsystems.flywheel.Flywheel;
import frc.team4276.frc2026.subsystems.hood.Hood;
import frc.team4276.frc2026.subsystems.intake.Intake;
import frc.team4276.frc2026.subsystems.spindexer.Spindexer;
import frc.team4276.frc2026.subsystems.turret.Turret;
import frc.team4276.frc2026.subsystems.vision.Vision;
import frc.team4276.lib.geometry.AllianceFlipUtil;
import frc.team4276.lib.hid.ViXController;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Intake intake;
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final ViXController controller;

  private boolean isFirstActive = false;

  private Supplier<ShootingParameters> shootingParams = ParamPreset.STOW::getParams;
  private ParamPreset currPreset = ParamPreset.STOW;

  private enum FeedState {
    NO,
    FERRY,
    ACTIVE
  }

  private FeedState feedState = FeedState.NO;

  private Trigger activeRumble = new Trigger(this::isHubActive);

  public Superstructure(
      Drive drive,
      Intake intake,
      Spindexer spindexer,
      Feeder feeder,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      Vision vision,
      ViXController controller) {
    this.drive = drive;
    this.intake = intake;
    this.spindexer = spindexer;
    this.feeder = feeder;
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.vision = vision;
    this.controller = controller;

    activeRumble
        .onTrue(this.controller.rumbleCommand(RumbleType.kBothRumble, 0.5, 0.25, 3))
        .onFalse(this.controller.rumbleCommand(RumbleType.kBothRumble, 0.5, 1.0, 1));
  }

  @Override
  public void periodic() {
    if (shooterAtSetpoint()) {
      if (feedState == FeedState.ACTIVE && isHubActive()) {
        feeder.setSystemState(Feeder.SystemState.FEED);
        spindexer.setSystemState(Spindexer.SystemState.GOGOGO);

      } else if (feedState == FeedState.FERRY) {
        feeder.setSystemState(Feeder.SystemState.FEED);
        spindexer.setSystemState(Spindexer.SystemState.GOGOGO);
      }

    } else {
      feeder.setSystemState(Feeder.SystemState.IDLE);
      spindexer.setSystemState(Spindexer.SystemState.IDLE);

    }

    turret.setPositionVelocity(shootingParams.get().turretAngle(), shootingParams.get().turretVelocity());
    hood.setPositionVelocity(shootingParams.get().hoodAngle(), shootingParams.get().hoodVelocity());
    flywheel.setVelocity(shootingParams.get().flywheelSpeed());

    Logger.recordOutput("Superstructure/IsFirstActive", isFirstActive);
    Logger.recordOutput("Superstructure/IsHubActive", isHubActive());
    Logger.recordOutput("Superstructure/FeedState", feedState);
    Logger.recordOutput("Superstructure/ShooterAtSetpoint", shooterAtSetpoint());
    Logger.recordOutput("Superstructure/ParamPreset", currPreset);

  }

  public void setIsFirstActive(boolean isFirstActive) {
    this.isFirstActive = isFirstActive;
  }

  public boolean isHubActive() {
    double matchTime = MatchState.getMatchTime();

    if (RobotState.isAutonomous() || matchTime > 130 || matchTime < 30) {
      return true;
    }

    if (matchTime > 105 || (matchTime < 80 && matchTime > 55)) {
      return isFirstActive;
    } else {
      return !isFirstActive;
    }
  }

  public boolean shooterAtSetpoint() {
    return turret.atSetpoint() && hood.atSetpoint() && flywheel.atSetpoint();
  }

  public Command deployIntake() {
    return Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.INTAKE));
  }

  public Command retractIntake() {
    return Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.RETRACT));
  }

  public Command enableShooter() { // auto aim
    return Commands.runOnce(() -> {
      if (RobotState.getInstance().getCurrentFieldZone() == FieldZone.ALLIANCE) {
        shootingParams = ShotCalculator.getInstance()::getHubParameters;

        feedState = FeedState.ACTIVE;

      } else {
        shootingParams = ShotCalculator.getInstance()::getFerryParameters;

        feedState = FeedState.FERRY;

      }
    })
    // .alongWith(
    //     Commands.waitSeconds(1.0)
    //         .andThen(Commands.runOnce(() -> drive.setVelocityScalar(DriveSpeedScalar.DEFAULT)))
    //         .finallyDo(() -> drive.setVelocityScalar(DriveSpeedScalar.CRAWL)))
            ;
  }

  public Command disableShooter() { // stop feeding; keep inertia and target
    return Commands.runOnce(() -> {
      if (RobotState.getInstance().getCurrentFieldZone() == FieldZone.ALLIANCE) {
        shootingParams = ShotCalculator.getInstance()::getHubParameters;

      } else {
        shootingParams = ShotCalculator.getInstance()::getFerryParameters;

      }

      feedState = FeedState.NO;

    });
  }

  public Command shootPreset(ParamPreset preset) { // rev up a few secs before active period; auto shoots once it begins
    return Commands.runOnce(() -> {
      currPreset = preset;
      shootingParams = currPreset::getParams;

      if (preset == ParamPreset.SHOWER || preset == ParamPreset.SHUB) {
        feedState = FeedState.ACTIVE;

        // drive.setHeadingAlignRotation(AllianceFlipUtil.apply(Rotation2d.kPi));

      } else if (preset == ParamPreset.SHERRY) {
        feedState = FeedState.FERRY;

        // drive.setHeadingAlignRotation(AllianceFlipUtil.apply(Rotation2d.kZero));

      }
    });
  }

  public Command turtle() { // go under trench
    return Commands.runOnce(() -> {
      intake.setWantedState(Intake.WantedState.INTAKE);
      currPreset = ParamPreset.TURTLE;
      shootingParams = currPreset::getParams;
      feedState = FeedState.NO;
    });
  }
}

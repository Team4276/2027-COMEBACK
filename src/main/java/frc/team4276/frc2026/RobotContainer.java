// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2026;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.team4276.frc2026.shooter.ShooterConstants.ParamPreset;
import frc.team4276.frc2026.subsystems.Superstructure;
import frc.team4276.frc2026.subsystems.drive.Drive;
import frc.team4276.frc2026.subsystems.drive.GyroIO;
import frc.team4276.frc2026.subsystems.drive.GyroIPigeon2;
import frc.team4276.frc2026.subsystems.drive.ModuleIO;
import frc.team4276.frc2026.subsystems.drive.ModuleIOKreo;
import frc.team4276.frc2026.subsystems.drive.ModuleIOSim;
import frc.team4276.frc2026.subsystems.feeder.Feeder;
import frc.team4276.frc2026.subsystems.feeder.FeederIO;
import frc.team4276.frc2026.subsystems.feeder.FeederIOTalonFX;
import frc.team4276.frc2026.subsystems.flywheel.Flywheel;
import frc.team4276.frc2026.subsystems.flywheel.FlywheelIO;
import frc.team4276.frc2026.subsystems.flywheel.FlywheelIOTalonFX;
import frc.team4276.frc2026.subsystems.hood.Hood;
import frc.team4276.frc2026.subsystems.hood.HoodIO;
import frc.team4276.frc2026.subsystems.hood.HoodIOTalonFX;
import frc.team4276.frc2026.subsystems.intake.Intake;
import frc.team4276.frc2026.subsystems.intake.IntakeIO;
import frc.team4276.frc2026.subsystems.intake.IntakeIOSpark;
import frc.team4276.frc2026.subsystems.spindexer.Spindexer;
import frc.team4276.frc2026.subsystems.spindexer.SpindexerIO;
import frc.team4276.frc2026.subsystems.spindexer.SpindexerIOSpark;
import frc.team4276.frc2026.subsystems.turret.Turret;
import frc.team4276.frc2026.subsystems.turret.TurretIO;
import frc.team4276.frc2026.subsystems.turret.TurretIOTalonFX;
import frc.team4276.frc2026.subsystems.vision.Vision;
import frc.team4276.frc2026.subsystems.vision.VisionIO;
import frc.team4276.frc2026.subsystems.vision.VisionIOPhotonVision;
import frc.team4276.lib.geometry.AllianceFlipUtil;
import frc.team4276.lib.hid.CowsController;
import frc.team4276.lib.hid.ViXController;

public class RobotContainer {
  private Drive drive;
  private Intake intake;
  private Spindexer spindexer;
  private Feeder feeder;
  private Turret turret;
  private Hood hood;
  private Flywheel flywheel;
  private Vision vision;

  private final Superstructure superstructure;

  private final ViXController driver = new ViXController(Ports.DRIVER_CONTROLLER);
  private final CowsController demoController = new CowsController(Ports.DEMO_CONTROLLER_LEFT,
      Ports.DEMO_CONTROLLER_RIGHT);

  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getType()) {
        case COMPBOT -> {
          // Real robot, instantiate hardware IO implementations
          drive = new Drive(
              Constants.isDemo ? demoController : driver,
              new GyroIPigeon2(),
              new ModuleIOKreo(0),
              new ModuleIOKreo(1),
              new ModuleIOKreo(2),
              new ModuleIOKreo(3));
          intake = new Intake(new IntakeIOSpark());
          spindexer = new Spindexer(new SpindexerIOSpark());
          feeder = new Feeder(new FeederIOTalonFX());
          turret = new Turret(new TurretIOTalonFX());
          hood = new Hood(new HoodIOTalonFX());
          flywheel = new Flywheel(new FlywheelIOTalonFX());
          vision = new Vision(RobotState.getInstance()::addVisionMeasurement, new VisionIOPhotonVision(0),
              new VisionIOPhotonVision(1));
        }

        case SIMBOT -> {
          // Sim robot, instantiate physics sim IO implementations
          drive = new Drive(
              Constants.isDemo ? demoController : driver,
              new GyroIO() {
              },
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
          intake = new Intake(new IntakeIO() {
          });
          spindexer = new Spindexer(new SpindexerIO() {
          });
          feeder = new Feeder(new FeederIO() {
          });
          turret = new Turret(new TurretIO() {
          });
          hood = new Hood(new HoodIO() {
          });
          flywheel = new Flywheel(new FlywheelIO() {
          });
          vision = new Vision(RobotState.getInstance()::addVisionMeasurement);
        }
      }
    }

    // No-op implmentations for replay
    if (drive == null) {
      drive = new Drive(
          Constants.isDemo ? demoController : driver,
          new GyroIO() {
          },
          new ModuleIO() {
          },
          new ModuleIO() {
          },
          new ModuleIO() {
          },
          new ModuleIO() {
          });
    }

    if (intake == null) {
      intake = new Intake(new IntakeIO() {
      });
    }

    if (spindexer == null) {
      spindexer = new Spindexer(new SpindexerIO() {
      });
    }

    if (feeder == null) {
      feeder = new Feeder(new FeederIO() {
      });
    }

    if (turret == null) {
      turret = new Turret(new TurretIO() {
      });
    }

    if (hood == null) {
      hood = new Hood(new HoodIO() {
      });
    }

    if (flywheel == null) {
      flywheel = new Flywheel(new FlywheelIO() {
      });
    }

    if (vision == null) {
      vision = new Vision(RobotState.getInstance()::addVisionMeasurement, new VisionIO() {
      }, new VisionIO() {
      });
    }

    superstructure = new Superstructure(drive, intake, spindexer, feeder, turret, hood, flywheel, vision, driver);

    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance()
                    .resetPose(
                        new Pose2d(
                            RobotState.getInstance().getEstimatedPose().getTranslation(),
                            AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    driver
        .rightTrigger()
        .whileTrue(superstructure.enableShooter());

    driver
        .rightBumper()
        .onTrue(superstructure.disableShooter());

    driver
        .leftTrigger()
        .onTrue(superstructure.deployIntake());

    driver
        .leftBumper()
        .onTrue(superstructure.retractIntake());

    driver
        .y()
        .onTrue(superstructure.shootPreset(ParamPreset.SHUB));

    driver
        .a()
        .onTrue(superstructure.shootPreset(ParamPreset.SHOWER));

    driver
        .x()
        .onTrue(superstructure.shootPreset(ParamPreset.SHERRY));

    driver
        .b()
        .onTrue(superstructure.turtle());

    driver
        .povUp()
        .onTrue(Commands.runOnce(() -> superstructure.setIsFirstActive(true)));

    driver
        .povDown()
        .onTrue(Commands.runOnce(() -> superstructure.setIsFirstActive(false)));

    // POV RIGHT/LEFT: adjust turret manually for zeroing
    // dashboard: zero turret
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}

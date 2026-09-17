package frc.team4276.frc2026.subsystems.drive;

import static frc.team4276.frc2026.subsystems.drive.DriveConstants.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command3.Mechanism;
import frc.team4276.frc2026.Constants;
import frc.team4276.frc2026.RobotState;
import frc.team4276.lib.dashboard.LoggedTunablePID;
import frc.team4276.lib.geometry.AllianceFlipUtil;
import frc.team4276.lib.hid.JoystickOutputController;

public class Drive implements Mechanism {
  public enum WantedState {
    TELEOP,
    PATH,
    HEADING_ALIGN,
    AUTO_ALIGN,
    IDLE,
    CHARACTERIZATION
  }

  public enum SystemState {
    TELEOP,
    PATH,
    HEADING_ALIGN,
    AUTO_ALIGN,
    IDLE,
    CHARACTERIZATION
  }

  private WantedState wantedState = WantedState.TELEOP;
  private SystemState systemState = SystemState.TELEOP;

  private final LoggedTunablePID teleopAutoAlignController = new LoggedTunablePID(
      3.0, 0, 0.1, Units.inchesToMeters(1.0), "Drive/AutoAlign/TeleopTranslation");
  private final LoggedTunablePID autoAutoAlignController = new LoggedTunablePID(
      3.0, 0, 0.1, Units.inchesToMeters(1.0), "Drive/AutoAlign/AutoTranslation");
  private final LoggedTunablePID headingAlignController = new LoggedTunablePID(20.0, 0, 0, Math.toRadians(1.0),
      "Drive/HeadingAlign");

  private Pose2d desiredAutoAlignPose = Pose2d.ZERO;
  private final double autoAlignStaticFrictionConstant = maxVelocityMPS * 0.02;

  private Rotation2d desiredHeadingAlignRotation = Rotation2d.ZERO;

  private double maxAutoAlignDriveTranslationOutput = maxVelocityMPS * 0.67;
  private double maxAutoAlignDriveRotationOutput = maxAngularVelocity;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private SwerveModulePosition[] lastModulePositions = null;
  private double lastTime = 0.0;

  private final JoystickOutputController controller;

  public enum DriveSpeedScalar {
    DEFAULT(1.0, 0.65),
    CRAWL(0.5, 0.65),
    DEMO(0.1, 0.1);

    private final double linearVelocityScalar;
    private final double angularVelocityScalar;

    private DriveSpeedScalar(double linearVelocityScalar, double angularVelocityScalar) {
      this.linearVelocityScalar = linearVelocityScalar;
      this.angularVelocityScalar = angularVelocityScalar;
    }

    public double linearVelocityScalar() {
      return linearVelocityScalar;
    }

    public double angularVelocityScalar() {
      return angularVelocityScalar;
    }
  }

  private DriveSpeedScalar driveSpeedScalar = Constants.isDemo ? DriveSpeedScalar.DEMO : DriveSpeedScalar.DEFAULT;

  // SmartDashboard no longer exists in 2027 - LoggedNetworkNumber is the replay-safe
  // NT-backed equivalent for a dashboard-editable value that's also read back.
  private final LoggedNetworkNumber customAlignX = new LoggedNetworkNumber("CustomAlignX", Double.NaN);
  private final LoggedNetworkNumber customAlignY = new LoggedNetworkNumber("CustomAlignY", Double.NaN);
  private final LoggedNetworkNumber customAlignRot = new LoggedNetworkNumber("CustomAlignRot", Double.NaN);

  public Drive(
      JoystickOutputController controller,
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.controller = controller;
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();
    PhoenixOdometryThread.getInstance().start();

    // Preserves the exact v2 behavior: this always ran every cycle regardless of
    // robot-enable state and was never gated by command scheduling (setWantedState()
    // is a plain field set, not a scheduled command). command3's Scheduler has no
    // built-in disabled-state gating of its own, so a sideload that runs
    // unconditionally every tick is the faithful equivalent - not a default command.
    getRegisteredScheduler().addPeriodic(this::periodic);
  }

  private void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (org.wpilib.driverstation.RobotState.isDisabled()) {
      // Stop moving when disabled
      for (var module : modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("Drive/SwerveStates/Torques", new SwerveModuleVelocity[] {});
      if (Constants.isTuning) {
        SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
        for (int i = 0; i < 4; i++) {
          states[i] = modules[i].getZeroHelperModuleState();
        }
        Logger.recordOutput("Drive/SwerveStates/ZeroHelper", states);
      }
    }

    updateOdom();

    systemState = handleStateTransition();
    Logger.recordOutput("Drive/SystemState", systemState);
    Logger.recordOutput("Drive/DesiredState", wantedState);
    applyState();

  }

  private void updateOdom() {
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
      }

      boolean includeMeasurement = true;
      if (lastModulePositions != null) {
        double dt = sampleTimestamps[i] - lastTime;
        for (int j = 0; j < modules.length; j++) {
          double velocity = (modulePositions[j].distance - lastModulePositions[j].distance) / dt;
          double omega = modulePositions[j].angle.minus(lastModulePositions[j].angle).getRadians() / dt;
          // Check if delta is too large
          if (Math.abs(velocity) > DriveConstants.maxVelocityMPS * 1.5
              || Math.abs(omega) > DriveConstants.maxAngularVelocity * 1.5) {
            includeMeasurement = false;
            break;
          }
        }
      }
      Logger.recordOutput("Drive/lastMeasurementIncluded", includeMeasurement);
      // If delta isn't too large we can include the measurement.
      if (includeMeasurement) {
        lastModulePositions = modulePositions;
        RobotState.getInstance()
            .addOdometryObservation(
                sampleTimestamps[i],
                gyroInputs.connected ? gyroInputs.yawPosition : null,
                modulePositions);
        lastTime = sampleTimestamps[i];
        RobotState.getInstance().addDriveSpeeds(kinematics.toChassisVelocities(getModuleStates()));
      }
    }
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case TELEOP -> SystemState.TELEOP;
      case PATH -> SystemState.PATH;
      case HEADING_ALIGN -> SystemState.HEADING_ALIGN;
      case AUTO_ALIGN -> SystemState.AUTO_ALIGN;
      case CHARACTERIZATION -> SystemState.CHARACTERIZATION;
      default -> SystemState.IDLE;
    };
  }

  private void applyState() {
    ChassisVelocities requestedSpeeds = new ChassisVelocities();

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

    Logger.recordOutput("RobotState/EstimatedPose", currentPose);
    Logger.recordOutput(
        "RobotState/EstimatedOdomPose", RobotState.getInstance().getEstimatedOdomPose());

    switch (systemState) {
      default:
        break;

      case TELEOP:
        requestedSpeeds = getJoystickRequestedSpeeds();

        break;

      case PATH:

        break;

      case HEADING_ALIGN:
        double headingAlignError = MathUtil.angleModulus(
            currentPose.getRotation().minus(desiredHeadingAlignRotation).getRadians());
        double headingAlignOmega = Math.min(
            headingAlignController.calculate(headingAlignError, 0.0),
            maxAutoAlignDriveRotationOutput);

        if (headingAlignController.atSetpoint()) {
          headingAlignOmega = 0.0;
        }

        requestedSpeeds = getJoystickRequestedSpeeds();
        requestedSpeeds.omega = headingAlignOmega;

        break;

      case AUTO_ALIGN:
        Translation2d translationError = desiredAutoAlignPose.getTranslation().minus(currentPose.getTranslation());
        double translationLinearError = translationError.getNorm();
        double translationLinearOutput;

        if (translationLinearError < teleopAutoAlignController.getErrorTolerance()) {
          translationLinearOutput = 0.0;

        } else if (org.wpilib.driverstation.RobotState.isAutonomous()) {
          translationLinearOutput = Math.abs(autoAutoAlignController.calculate(translationLinearError, 0.0))
              + autoAlignStaticFrictionConstant;

        } else {
          translationLinearOutput = Math.abs(teleopAutoAlignController.calculate(translationLinearError, 0.0))
              + autoAlignStaticFrictionConstant;
        }

        translationLinearOutput = Math.min(translationLinearOutput, maxAutoAlignDriveTranslationOutput);

        // Translation2d.getAngle() now returns Optional<Rotation2d> (undefined when
        // translationError is exactly zero) - translationLinearOutput is already zeroed in
        // that case above, so any fallback angle here is multiplied by ~0 regardless.
        Rotation2d translationErrorAngle = translationError.getAngle().orElse(Rotation2d.ZERO);
        double vx = translationLinearOutput * translationErrorAngle.getCos();
        double vy = translationLinearOutput * translationErrorAngle.getSin();

        double autoAlignThetaError = MathUtil.angleModulus(
            currentPose.getRotation().minus(desiredAutoAlignPose.getRotation()).getRadians());
        double omega = Math.min(
            headingAlignController.calculate(autoAlignThetaError, 0.0),
            maxAutoAlignDriveRotationOutput);

        if (headingAlignController.atSetpoint()) {
          omega = 0.0;
        }

        requestedSpeeds = new ChassisVelocities(vx, vy, omega);

        break;
      case CHARACTERIZATION:
        break;
    }

    requestedSpeeds = requestedSpeeds.toRobotRelative(currentPose.getRotation());

    SwerveModuleVelocity[] setpointStates;
    ChassisVelocities setpointSpeeds;

    setpointSpeeds = requestedSpeeds.discretize(0.02);
    setpointStates = kinematics.toSwerveModuleVelocities(setpointSpeeds);
    setpointStates = SwerveDriveKinematics.desaturateWheelVelocities(setpointStates, maxVelocityMPS);

    // Send setpoints to modules. runSetpoint() returns the optimized state it actually
    // applied (SwerveModuleVelocity.optimize() returns a new instance rather than mutating
    // in place, unlike the old SwerveModuleState.optimize()), so capture it back into
    // setpointStates for the logging below to reflect what was actually commanded.
    for (int i = 0; i < 4; i++) {
      setpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints
    Logger.recordOutput("Drive/RequestedSpeeds", requestedSpeeds);
    Logger.recordOutput("Drive/SetpointSpeeds", setpointSpeeds);
    Logger.recordOutput(
        "Drive/SwerveStates/UnoptimizedSetpoints",
        kinematics.toSwerveModuleVelocities(requestedSpeeds.discretize(0.02)));
    Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
  }

  private ChassisVelocities getJoystickRequestedSpeeds() {
    double linearMagnitude = Math.hypot(-controller.getLeftWithDeadband().y, -controller.getLeftWithDeadband().x);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    Translation2d linearVelocity = Translation2d.ZERO;

    if (linearMagnitude > 1e-6) {
      linearVelocity = new Translation2d(
          linearMagnitude,
          new Rotation2d(
              controller.getLeftWithDeadband().y, controller.getLeftWithDeadband().x))
          .times(driveSpeedScalar.linearVelocityScalar);
    }

    // Square rotation value for more precise control
    double omega = Math.copySign(
        controller.getRightWithDeadband().x * controller.getRightWithDeadband().x,
        -controller.getRightWithDeadband().x)
        * driveSpeedScalar.angularVelocityScalar;

    return new ChassisVelocities(
            linearVelocity.getX() * DriveConstants.maxVelocityMPS,
            linearVelocity.getY() * DriveConstants.maxVelocityMPS,
            omega * DriveConstants.maxAngularVelocity)
        .toRobotRelative(AllianceFlipUtil.apply(Rotation2d.k180deg));
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void setAutoAlignPose(Pose2d pose) {
    setWantedState(WantedState.AUTO_ALIGN);
    desiredAutoAlignPose = pose;
  }

  public void setAutoAlignCustom() {
    double x = customAlignX.get();
    double y = customAlignY.get();
    double rot = customAlignRot.get();

    if (x != Double.NaN && y != Double.NaN && rot != Double.NaN) {
      setAutoAlignPose(new Pose2d(x, y, Rotation2d.fromDegrees(rot)));
    }
  }

  public void setHeadingAlignRotation(Rotation2d rotation) {
    setWantedState(WantedState.HEADING_ALIGN);
    desiredHeadingAlignRotation = rotation;
  }

  public void alignToHub() {
    setHeadingAlignRotation(RobotState.getInstance().getHubAlignHeading());
  }

  public void setVelocityScalar(DriveSpeedScalar scalar){
    driveSpeedScalar = scalar;
  }
}

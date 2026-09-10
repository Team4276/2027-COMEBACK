package frc.team4276.frc2026;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import frc.team4276.frc2026.FieldConstants.FieldZone;
import frc.team4276.frc2026.subsystems.vision.VisionConstants;
import frc.team4276.lib.dashboard.LoggedTunableNumber;
import frc.team4276.lib.geometry.GeomUtil;

import static frc.team4276.frc2026.subsystems.drive.DriveConstants.kinematics;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  private Rotation2d lastYaw = Rotation2d.kZero;

  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      lastYaw,
      lastWheelPositions,
      Pose2d.kZero,
      VecBuilder.fill(0.1, 0.1, 0.1), // TODO tune
      VecBuilder.fill(.9, .9, 2.0));

  private SwerveDrivePoseEstimator odomPoseEstimator = new SwerveDrivePoseEstimator(kinematics, lastYaw,
      lastWheelPositions, Pose2d.kZero);

  private static final double poseBufferSizeSec = 2.0;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

  private Set<FuelPoseRecord> fuelPoses = new HashSet<>();
  private LoggedTunableNumber fuelOverlap = new LoggedTunableNumber("RobotState/FuelOverlap", 0.5);

  private ChassisVelocities robotVelocity = new ChassisVelocities();

  @AutoLogOutput
  private int visionUpdateCount = 0;

  private double lastUsedVisionPoseEstimateTimestamp = 0.0;

  private static RobotState mInstance;

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  private RobotState() {
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(lastYaw, lastWheelPositions, pose);
    odomPoseEstimator.resetPosition(lastYaw, lastWheelPositions, pose);
  }

  public void addDriveSpeeds(ChassisVelocities speeds) {
    robotVelocity = speeds;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    if (yaw == null) {
      var twist = kinematics.toTwist2d(lastWheelPositions, wheelPositions);
      yaw = odomPoseEstimator.getEstimatedPosition().getRotation().rotateBy(new Rotation2d(twist.dtheta));
    }
    poseEstimator.updateWithTime(timestamp, yaw, wheelPositions);
    odomPoseEstimator.updateWithTime(timestamp, yaw, wheelPositions);
    lastWheelPositions = wheelPositions;
    lastYaw = yaw;
    poseBuffer.addSample(timestamp, getEstimatedOdomPose());
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    visionUpdateCount++;

    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

    lastUsedVisionPoseEstimateTimestamp = timestampSeconds;
  }

  public void addObjectMeasurement(FuelTxTyObservation observation) {
    var oldOdometryPose = poseBuffer.getSample(observation.timestamp());
    if (oldOdometryPose.isEmpty()) {
      return;
    }
    Pose2d fieldToRobot = poseEstimator.getEstimatedPosition()
        .transformBy(new Transform2d(odomPoseEstimator.getEstimatedPosition(), oldOdometryPose.get()));
    Transform3d robotToCamera = VisionConstants.configs[observation.camera()].robotToCamera;

    // Assume coral height of zero and find midpoint of width of bottom tx ty
    double tx = (observation.tx()[2] + observation.tx()[3]) / 2;
    double ty = (observation.ty()[2] + observation.ty()[3]) / 2;

    double cameraToCoralAngle = -robotToCamera.getRotation().getY() - ty;
    if (cameraToCoralAngle >= 0) {
      return;
    }

    double cameraToFuelNorm = (-robotToCamera.getZ())
        / Math.tan(-robotToCamera.getRotation().getY() - ty)
        / Math.cos(-tx);
    Pose2d fieldToCamera = fieldToRobot.transformBy(GeomUtil.toTransform2d(robotToCamera));
    Pose2d fieldToFuel = fieldToCamera
        .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-tx)))
        .transformBy(
            new Transform2d(new Translation2d(cameraToFuelNorm, 0), Rotation2d.kZero));
    Translation2d fieldToFuelTranslation2d = fieldToFuel.getTranslation();
    FuelPoseRecord fuelPoseRecord = new FuelPoseRecord(fieldToFuelTranslation2d, observation.timestamp());

    fuelPoses = fuelPoses.stream()
        .filter(
            (x) -> x.translation.getDistance(fieldToFuelTranslation2d) > fuelOverlap.get())
        .collect(Collectors.toSet());
    fuelPoses.add(fuelPoseRecord);
  }

  public double getLastUsedVisionPoseEstimateTimestamp() {
    return lastUsedVisionPoseEstimateTimestamp;
  }

  public Rotation2d getHubAlignHeading() {
    return FieldConstants.Hub.innerCenterPoint.toTranslation2d().minus(getEstimatedPose().getTranslation()).getAngle();
  }

  public FieldZone getCurrentFieldZone(){
    double x = getEstimatedPose().getX();

    if(x < FieldConstants.LinesVertical.allianceZone){
      return FieldZone.ALLIANCE;

    } else if(x < FieldConstants.LinesVertical.neutralZoneFar){
      return FieldZone.MIDDLE;

    } else {
      return FieldZone.OPPOSING;

    }
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/EstimatedOdomPose")
  public Pose2d getEstimatedOdomPose() {
    return odomPoseEstimator.getEstimatedPosition();
  }

  public Optional<Pose2d> getEstimatedOdomPoseAtTime(double timestamp) {
    return odomPoseEstimator.sampleAt(timestamp);
  }

  public ChassisVelocities getFieldVelocity() {
    return ChassisVelocities.fromRobotRelativeSpeeds(robotVelocity, getEstimatedPose().getRotation());
  }

  public record FuelPoseRecord(Translation2d translation, double timestamp) {
  }

  public record FuelTxTyObservation(int camera, double[] tx, double[] ty, double timestamp) {
  }
}

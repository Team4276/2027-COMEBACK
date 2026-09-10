package frc.team4276.lib.geometry;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.team4276.frc2026.Constants;
import frc.team4276.frc2026.FieldConstants;

public class AllianceFlipUtil {
  static {
    SmartDashboard.putBoolean("Sim/OverrideFlip", false);
  }

  private static boolean overrideFlip = true;

  public static double flipX(double x) {
    return FieldConstants.fieldLength - x;
  }

  public static double flipY(double y) {
    return FieldConstants.fieldWidth - y;
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return shouldFlip()
        ? new Translation2d(applyX(translation.getX()), applyY(translation.getY()))
        : translation;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  /**
   * For SIM true sets to blue alliance
   *
   * @param shouldOverrideFlip
   */
  public static void overrideFlip(boolean shouldOverrideFlip) {
    overrideFlip = shouldOverrideFlip;
  }

  public static boolean shouldFlip() {
    overrideFlip = SmartDashboard.getBoolean("Sim/OverrideFlip", overrideFlip);

    return MatchState.getAlliance().isPresent()
        && MatchState.getAlliance().get() == Alliance.RED
        && (Constants.isSim ? !overrideFlip : true);
  }
}

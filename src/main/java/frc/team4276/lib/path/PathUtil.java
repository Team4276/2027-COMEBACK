package frc.team4276.lib.path;

import static frc.team4276.frc2026.FieldConstants.*;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class PathUtil {
  public static Translation2d mirrorLengthwise(Translation2d trans) {
    return new Translation2d(trans.getX(), fieldWidth - trans.getY());
  }

  public static Rotation2d mirrorLengthwise(Rotation2d trans) {
    return trans.unaryMinus();
  }

  public static Pose2d mirrorLengthwise(Pose2d pose) {
    return new Pose2d(
        mirrorLengthwise(pose.getTranslation()), mirrorLengthwise(pose.getRotation()));
  }

  public static ChassisVelocities mirrorLengthwise(ChassisVelocities speeds) {
    return new ChassisVelocities(
        speeds.vx,
        -1.0 * speeds.vy,
        -1.0 * speeds.omega);
  }
}

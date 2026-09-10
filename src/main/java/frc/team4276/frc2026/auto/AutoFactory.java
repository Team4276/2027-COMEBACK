package frc.team4276.frc2026.auto;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.team4276.frc2026.FieldConstants;
import frc.team4276.frc2026.RobotContainer;
import frc.team4276.frc2026.RobotState;
import frc.team4276.lib.dashboard.Elastic;
import frc.team4276.lib.dashboard.Elastic.Notification;
import frc.team4276.lib.dashboard.Elastic.Notification.NotificationLevel;
import frc.team4276.lib.geometry.AllianceFlipUtil;

import java.util.List;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class AutoFactory {
  private RobotContainer robotContainer;

  public AutoFactory(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
  }

  Command idle() {
    return resetPose(
        new Pose2d(
            RobotState.getInstance().getEstimatedPose().getTranslation(),
            AllianceFlipUtil.apply(Rotation2d.kZero)));
  }

  void autoEnd(){
    
  }

  private Command resetPose(Pose2d pose) {
    return Commands.runOnce(() -> RobotState.getInstance().resetPose(pose));
  }

  // private Command driveTrajectory(Trajectory<SwerveSample> traj) {
  //   ElasticUI.putAutoTrajectory(traj);
  //   ElasticUI.putAutoPath(
  //       List.of(traj.getInitialPose(false).get(), traj.getFinalPose(false).get()));

  //   return Commands.runOnce(
  //           () -> {
  //             robotContainer.getDrive().setChoreoTrajectory(traj);
  //             RobotState.getInstance().setVisionMode(VisionMode.REJECT_ALL);
  //           })
  //       .andThen(Commands.waitUntil(() -> robotContainer.getDrive().isTrajectoryFinished()))
  //       .finallyDo(
  //           () -> {
  //             RobotState.getInstance().setVisionMode(VisionMode.ACCEPT_ALL);
  //           });
  // }

  // private Command driveToPoint(Pose2d pose) {
  //   return driveToPoint(() -> pose);
  // }

  // private Command driveToPoint(Supplier<Pose2d> pose) {
  //   return Commands.run(() -> robotContainer.getDrive().setAutoAlignPose(pose.get()))
  //       .until(() -> robotContainer.getDrive().isAtAutoAlignPose());
  // }

  // private Command driveToPointWithCheckerPose(Supplier<Pose2d> pose, Pose2d checkerPose) {
  //   return Commands.run(
  //           () -> {
  //             robotContainer.getDrive().setAutoAlignPose(pose.get());
  //             robotContainer.getDrive().setIsAutoAlignCheckPose(checkerPose);
  //           })
  //       .until(() -> robotContainer.getDrive().isAtAutoAlignPose());
  // }

  /**
   * Returns whether robot has crossed x boundary, accounting for alliance flip
   *
   * @param xPosition X position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed x coordinate towards center line or away
   *     from center line
   */
  private boolean xCrossed(double xPosition, boolean towardsCenterline) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsCenterline) {
        return robotPose.getX() < FieldConstants.fieldLength - xPosition;
      } else {
        return robotPose.getX() > FieldConstants.fieldLength - xPosition;
      }
    } else {
      if (towardsCenterline) {
        return robotPose.getX() > xPosition;
      } else {
        return robotPose.getX() < xPosition;
      }
    }
  }

  /** Command that waits for x boundary to be crossed. See {@link #xCrossed(double, boolean)} */
  private Command waitUntilXCrossed(double xPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenterline));
  }

  /**
   * Returns whether robot has crossed y boundary, accounting for alliance flip
   *
   * @param yPosition Y position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed y coordinate towards center line or away
   *     from center line
   */
  private boolean yCrossed(double yPosition, boolean towardsCenterline) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsCenterline) {
        return robotPose.getY() < FieldConstants.fieldWidth - yPosition;
      } else {
        return robotPose.getY() > FieldConstants.fieldWidth - yPosition;
      }
    } else {
      if (towardsCenterline) {
        return robotPose.getY() > yPosition;
      } else {
        return robotPose.getY() < yPosition;
      }
    }
  }

  /** Command that waits for y boundary to be crossed. See {@link #yCrossed(double, boolean)} */
  private Command waitUntilYCrossed(double yPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> yCrossed(yPosition, towardsCenterline));
  }

  private Command printCommand(String text) {
    return Commands.runOnce(() -> System.out.println(text));
  }

  private Command notificationCommand(String notification) {
    return notificationCommand(
        new Notification(NotificationLevel.INFO, "Auto Action", notification, 3000));
  }

  private Command notificationCommand(Notification notification) { // Jank but gud enough for now
    return Commands.runOnce(() -> Elastic.sendNotification(notification));
  }
}

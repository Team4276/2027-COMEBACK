package frc.team4276.frc2026.auto;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.team4276.lib.VirtualSubsystem;
import frc.team4276.lib.geometry.AllianceFlipUtil;

import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoSelector extends VirtualSubsystem {
  private final AutoFactory autoFactory;

  private final LoggedDashboardChooser<Supplier<Command>> routineChooser =
      new LoggedDashboardChooser<>("Comp/Auto/RoutineChooser");
  private Supplier<Command> lastRoutine = () -> Commands.none();
  private String lastRoutineName = "";

  private static boolean autoChanged = true;

  private final LoggedNetworkNumber delayInput = new LoggedNetworkNumber("Comp/Auto/Delay", 0.0);

  public AutoSelector(AutoFactory autoFactory) {
    this.autoFactory = autoFactory;

    routineChooser.addDefaultOption("Do Nothing", () -> this.autoFactory.idle());
  }

  /** Returns the selected auto command with the inputted delay. */
  public Command getCommand() {
    return lastRoutine
        .get()
        .beforeStarting(Commands.waitSeconds(getDelayInput()))
        .finallyDo(() -> this.autoFactory.autoEnd());
  }

  public double getDelayInput() {
    return delayInput.get();
  }

  private boolean wasRed = false;

  public void periodic() {
    // Skip updates when actively running in auto
    if (RobotState.isAutonomousEnabled() && lastRoutine != null) {
      return;
    }

    SmartDashboard.putNumber("Comp/Auto/Delay Input Submitted ", getDelayInput());

    // Update the list of questions
    var routineName = routineChooser.getSendableChooser().getSelected();

    // Update the routine and responses
    if (lastRoutineName != routineName) {
      var selectedRoutine = routineChooser.get();
      if (selectedRoutine == null) {
        return;
      }

      lastRoutine = selectedRoutine;
      lastRoutineName = routineName;
      autoChanged = true;
    }

    SmartDashboard.putString("Comp/Auto/Routine Submitted ", lastRoutineName);

    if (AllianceFlipUtil.shouldFlip() != wasRed) {
      autoChanged = true;
    }

    wasRed = AllianceFlipUtil.shouldFlip();
  }

  public static boolean hasAutoChanged() {
    if (autoChanged) {
      autoChanged = false;
      return true;
    }

    return false;
  }
}

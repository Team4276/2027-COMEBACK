package frc.team4276.frc2026.auto;

import org.wpilib.driverstation.RobotState;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import frc.team4276.lib.VirtualSubsystem;
import frc.team4276.lib.geometry.AllianceFlipUtil;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoSelector extends VirtualSubsystem {
  private final AutoFactory autoFactory;

  // LoggedDashboardChooser was renamed to LoggedNetworkChooser and no longer exposes the
  // underlying dashboard widget (no getSendableChooser()), so option names are tracked
  // separately here for the "what changed" / telemetry logic below.
  private final LoggedNetworkChooser<Supplier<Command>> routineChooser =
      new LoggedNetworkChooser<>("Comp/Auto/RoutineChooser");
  private final Map<Supplier<Command>, String> routineNames = new HashMap<>();
  private Supplier<Command> lastRoutine =
      () -> Command.noRequirements(coroutine -> {}).named("None");
  private String lastRoutineName = "";

  private static boolean autoChanged = true;

  private final LoggedNetworkNumber delayInput = new LoggedNetworkNumber("Comp/Auto/Delay", 0.0);

  public AutoSelector(AutoFactory autoFactory) {
    this.autoFactory = autoFactory;

    addDefaultOption("Do Nothing", () -> this.autoFactory.idle());
  }

  private void addDefaultOption(String name, Supplier<Command> routine) {
    routineNames.put(routine, name);
    routineChooser.addDefault(name, routine);
  }

  /**
   * Returns the selected auto command with the inputted delay. {@code autoFactory.autoEnd()} is
   * called whether the routine finishes naturally (end of the command body) or is canceled early
   * (e.g. driver switches to teleop) - command3 has no direct .finallyDo() equivalent, so both
   * exit paths are handled explicitly: normal completion inline, cancellation via
   * whenCanceled().
   */
  public Command getCommand() {
    Supplier<Command> routine = lastRoutine;
    double delaySeconds = getDelayInput();

    return Command.noRequirements(coroutine -> {
      coroutine.wait(Seconds.of(delaySeconds));
      coroutine.await(routine.get());
      autoFactory.autoEnd();
    })
        .whenCanceled(() -> autoFactory.autoEnd())
        .named("Auto");
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

    Logger.recordOutput("Comp/Auto/Delay Input Submitted ", getDelayInput());

    // Update the routine and responses
    var selectedRoutine = routineChooser.get();

    if (lastRoutine != selectedRoutine) {
      if (selectedRoutine == null) {
        return;
      }

      lastRoutine = selectedRoutine;
      lastRoutineName = routineNames.getOrDefault(selectedRoutine, "");
      autoChanged = true;
    }

    Logger.recordOutput("Comp/Auto/Routine Submitted ", lastRoutineName);

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

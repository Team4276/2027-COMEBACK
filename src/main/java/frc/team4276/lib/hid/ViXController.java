package frc.team4276.lib.hid;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandXboxController;

public class ViXController extends CommandXboxController implements JoystickOutputController {
  private double JOYSTICK_DEADBAND = 0.1;
  private double TRIGGER_DEADBAND = 0.25;

  public ViXController(int port) {
    super(port);
  }

  public ViXController(int port, double deadband) {
    super(port);
    this.JOYSTICK_DEADBAND = deadband;
  }

  public void setDeadband(double deadband) {
    this.JOYSTICK_DEADBAND = deadband;
  }

  @Override
  public JoystickOutput getRightWithDeadband() {
    return Math.hypot(getRightX(), getRightY()) < JOYSTICK_DEADBAND
        ? new JoystickOutput()
        : getRight();
  }

  @Override
  public JoystickOutput getRight() {
    return new JoystickOutput(getRightX(), getRightY());
  }

  @Override
  public JoystickOutput getLeftWithDeadband() {
    return Math.hypot(getLeftX(), getLeftY()) < JOYSTICK_DEADBAND
        ? new JoystickOutput()
        : getLeft();
  }

  @Override
  public JoystickOutput getLeft() {
    return new JoystickOutput(getLeftX(), getLeftY());
  }

  // CommandXboxController.getHID() now covariantly returns CommandGenericHID (not the raw
  // GenericHID), so reaching the raw POV reading needs one more .getHID() hop - and getPOV()
  // itself now returns the POVDirection enum instead of a raw int angle in degrees.
  public boolean getPOVUP() {
    return getHID().getHID().getPOV() == POVDirection.UP;
  }

  public boolean getPOVRIGHT() {
    return getHID().getHID().getPOV() == POVDirection.RIGHT;
  }

  public boolean getPOVDOWN() {
    return getHID().getHID().getPOV() == POVDirection.DOWN;
  }

  public boolean getPOVLEFT() {
    return getHID().getHID().getPOV() == POVDirection.LEFT;
  }

  public boolean getLT() {
    return getLeftTrigger() > TRIGGER_DEADBAND;
  }

  public boolean getRT() {
    return getRightTrigger() > TRIGGER_DEADBAND;
  }

  /**
   * Rumbles the given motor(s) once. Pass multiple types (e.g. {@code LEFT_RUMBLE,
   * RIGHT_RUMBLE}) to rumble them together - v2's {@code RumbleType.kBothRumble} no longer
   * exists in 2027, since the DS now models left/right main + left/right trigger motors
   * individually.
   */
  public Command rumbleCommand(double value, double duration, RumbleType... types) {
    return rumbleCommand(value, duration, 1, types);
  }

  public Command rumbleCommand(double value, double duration, int times, RumbleType... types) {
    Command[] pulses = new Command[times];
    for (int i = 0; i < times; i++) {
      pulses[i] = singlePulse(value, duration, types);
    }
    return Command.sequence(pulses).named("Rumble");
  }

  private Command singlePulse(double value, double duration, RumbleType... types) {
    return Command.sequence(
        Command.noRequirements(coroutine -> setRumble(value, types)).named("Rumble[On]"),
        Command.waitFor(Seconds.of(duration)).named("Rumble[Hold]"),
        Command.noRequirements(coroutine -> setRumble(0.0, types)).named("Rumble[Off]"),
        Command.waitFor(Seconds.of(0.1)).named("Rumble[Gap]"))
        .named("Rumble[Pulse]");
  }

  private void setRumble(double value, RumbleType... types) {
    // CommandXboxController no longer extends CommandGenericHID (composition, not
    // inheritance, in v3) so setRumble() has to be reached through the composed getHID().
    for (RumbleType type : types) {
      getHID().setRumble(type, value);
    }
  }
}

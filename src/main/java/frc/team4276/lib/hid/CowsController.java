package frc.team4276.lib.hid;

import org.wpilib.driverstation.Joystick;

public class CowsController implements JoystickOutputController {
  private double JOYSTICK_DEADBAND = 0.1;

  private final Joystick leftStick;
  private final Joystick rightStick;

  public CowsController(int left, int right) {
    leftStick = new Joystick(left);
    rightStick = new Joystick(right);
  }

  public CowsController withDeadband(double deadband) {
    this.JOYSTICK_DEADBAND = deadband;

    return this;
  }

  public Joystick getRightStick() {
    return rightStick;
  }

  public Joystick getLeftStick() {
    return leftStick;
  }

  @Override
  public JoystickOutput getRightWithDeadband() {
    return Math.hypot(rightStick.getX(), rightStick.getY()) < JOYSTICK_DEADBAND
        ? new JoystickOutput()
        : getRight();
  }

  /** No Deadband */
  @Override
  public JoystickOutput getRight() {
    return new JoystickOutput(rightStick.getX(), rightStick.getY());
  }

  @Override
  public JoystickOutput getLeftWithDeadband() {
    return Math.hypot(leftStick.getX(), leftStick.getY()) < JOYSTICK_DEADBAND
        ? new JoystickOutput()
        : getLeft();
  }

  @Override
  public JoystickOutput getLeft() {
    return new JoystickOutput(leftStick.getX(), leftStick.getY());
  }

  public boolean getLT() {
    return leftStick.getTrigger();
  }

  public boolean getRT() {
    return rightStick.getTrigger();
  }
}

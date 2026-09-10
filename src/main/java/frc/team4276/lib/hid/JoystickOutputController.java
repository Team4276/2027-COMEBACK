package frc.team4276.lib.hid;

public interface JoystickOutputController {

  default JoystickOutput getRight() {
    return new JoystickOutput();
  }

  default JoystickOutput getRightWithDeadband() {
    return new JoystickOutput();
  }

  default JoystickOutput getLeft() {
    return new JoystickOutput();
  }

  default JoystickOutput getLeftWithDeadband() {
    return new JoystickOutput();
  }
}

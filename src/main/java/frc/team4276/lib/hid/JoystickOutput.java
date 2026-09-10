package frc.team4276.lib.hid;

public class JoystickOutput {
  public final double x;
  public final double y;

  public JoystickOutput(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public JoystickOutput() {
    this.x = 0.0;
    this.y = 0.0;
  }
}

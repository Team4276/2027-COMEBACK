package frc.team4276.lib.dashboard;

import org.wpilib.math.controller.PIDController;
import frc.team4276.frc2026.Constants;

public class LoggedTunablePID extends PIDController {
  public final LoggedTunableNumber Kp;
  public final LoggedTunableNumber Ki;
  public final LoggedTunableNumber Kd;
  public final LoggedTunableNumber KTol;

  private final String key;

  public LoggedTunablePID(double kp, double ki, double kd, String key) {
    super(kp, ki, kd);
    this.key = key;
    KTol = new LoggedTunableNumber(this.key + "/Tolerance", getErrorTolerance());
    Kd = new LoggedTunableNumber(this.key + "/kD", kd);
    Ki = new LoggedTunableNumber(this.key + "/kI", ki);
    Kp = new LoggedTunableNumber(this.key + "/kP", kp);
  }

  public LoggedTunablePID(double kp, double ki, double kd, double tol, String key) {
    super(kp, ki, kd);
    this.key = key;
    KTol = new LoggedTunableNumber(this.key + "/Tolerance", tol);
    Kd = new LoggedTunableNumber(this.key + "/kD", kd);
    Ki = new LoggedTunableNumber(this.key + "/kI", ki);
    Kp = new LoggedTunableNumber(this.key + "/kP", kp);
  }

  @Override
  public double calculate(double measurement, double setpoint) {
    if (Constants.isTuning) {
      setPID(Kp.getAsDouble(), Ki.getAsDouble(), Kd.getAsDouble());
      setTolerance(KTol.getAsDouble());
    }
    return super.calculate(measurement, setpoint);
  }
}

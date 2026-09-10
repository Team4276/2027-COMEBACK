package frc.team4276.lib.dashboard;

import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.trajectory.TrapezoidProfile;
import frc.team4276.frc2026.Constants;

public class LoggedTunableProfiledPID extends ProfiledPIDController {
  public final LoggedTunableNumber Kp;
  public final LoggedTunableNumber Ki;
  public final LoggedTunableNumber Kd;
  public final LoggedTunableNumber KTol;
  public final LoggedTunableNumber maxVel;
  public final LoggedTunableNumber maxAccel;

  private final String key;

  public LoggedTunableProfiledPID(
      String key, double kp, double ki, double kd, double maxVel, double maxAccel) {
    super(kp, ki, kd, new TrapezoidProfile.Constraints(maxVel, maxAccel));
    this.key = key;
    this.maxAccel = new LoggedTunableNumber(this.key + "/MaxAccel", maxAccel);
    this.maxVel = new LoggedTunableNumber(this.key + "/MaxVel", maxVel);
    KTol = new LoggedTunableNumber(this.key + "/Tolerance", getPositionTolerance());
    Kd = new LoggedTunableNumber(this.key + "/kD", kd);
    Ki = new LoggedTunableNumber(this.key + "/kI", ki);
    Kp = new LoggedTunableNumber(this.key + "/kP", kp);
  }

  public LoggedTunableProfiledPID(
      String key, double kp, double ki, double kd, double tol, double maxVel, double maxAccel) {
    super(kp, ki, kd, new TrapezoidProfile.Constraints(maxVel, maxAccel));
    this.key = key;
    this.maxAccel = new LoggedTunableNumber(this.key + "/MaxAccel", maxAccel);
    this.maxVel = new LoggedTunableNumber(this.key + "/MaxVel", maxVel);
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
      setConstraints(
          new TrapezoidProfile.Constraints(maxVel.getAsDouble(), maxAccel.getAsDouble()));
    }

    return super.calculate(measurement, setpoint);
  }
}

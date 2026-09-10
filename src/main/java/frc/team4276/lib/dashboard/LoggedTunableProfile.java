package frc.team4276.lib.dashboard;

import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import frc.team4276.frc2026.Constants;

public class LoggedTunableProfile {
  private TrapezoidProfile profile;
  public final LoggedTunableNumber maxVel;
  public final LoggedTunableNumber maxAccel;

  private final String key;

  public LoggedTunableProfile(String key, double maxVel, double maxAccel) {
    profile = new TrapezoidProfile(new Constraints(maxVel, maxAccel));
    this.key = key;
    this.maxAccel = new LoggedTunableNumber(this.key + "/MaxAccel", maxAccel);
    this.maxVel = new LoggedTunableNumber(this.key + "/MaxVel", maxVel);
  }

  public State calculate(double t, State current, State goal) {
    if (Constants.isTuning) {
      profile = new TrapezoidProfile(new Constraints(maxVel.getAsDouble(), maxAccel.getAsDouble()));
    }

    return profile.calculate(t, current, goal);
  }
}

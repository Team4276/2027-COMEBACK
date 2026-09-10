package frc.team4276.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = true;

    public double positionRev = 0.0;
    public double absolutePositionRad = 0.0;

    public double appliedVolts = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;

    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void setOpenLoop(double volts) {
  }

  public default void setPosition(double position) {
  }

  public default void setBrakeMode(boolean enable) {
  }
}

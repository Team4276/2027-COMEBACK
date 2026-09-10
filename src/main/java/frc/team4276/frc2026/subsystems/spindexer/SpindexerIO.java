package frc.team4276.frc2026.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public boolean connected = true;

    public double appliedVolts = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;

    public double tempCelsius = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {
  }

  public default void setOpenLoop(double voltage) {
  }

  public default void setBrakeMode(boolean enable) {
  }
}

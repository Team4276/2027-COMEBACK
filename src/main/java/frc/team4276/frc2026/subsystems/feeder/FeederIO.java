package frc.team4276.frc2026.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean connected = true;

    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrent = 0.0;

    public double tempCelsius = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {
  }

  public default void setOpenLoop(double voltage) {
  }

  public default void setBrakeMode(boolean enable) {
  }
}

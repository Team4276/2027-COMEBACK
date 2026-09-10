package frc.team4276.frc2026.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean[] connected = { true, true };
    public double[] appliedVolts = { 0.0, 0.0 };
    public double[] supplyCurrent = { 0.0, 0.0 };
    public double[] statorCurrent = { 0.0, 0.0 };
    public double[] tempCelsius = { 0.0, 0.0 };

    public double[] velocityRPS = { 0.0, 0.0 }; // rpm

    public double[][] queueTimestamps = new double[][] {};
    public double[][] queueVelocitiesRPS = new double[][] {};
  }

  public default void updateInputs(FlywheelIOInputs inputs) {
  }

  public default void setRpm(double primary, double hood) {
  }

  public default void setOpenLoop(double primaryVolts, double hoodVolts) {
  }

  public default void setBrakeMode(boolean enable) {
  }
}

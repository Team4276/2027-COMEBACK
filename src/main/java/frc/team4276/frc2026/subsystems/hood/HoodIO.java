package frc.team4276.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public boolean connected = false;
        public double positionRev = 0.0;
        public double absolutePositionRad = 0.0;

        public double velocityRPS = 0.0;

        public double appliedVolts = 0.0;
        public double supplyCurrent = 0.0;
        public double statorCurrent = 0.0;

        public double tempCelsius = 0.0;

        public double[] queueTimestamps = new double[] {};
        public double[] queuePositionRev = new double[] {};
    }

    public default void updateInputs(HoodIOInputs inputs) {
    }

    public default void setPositionVelocity(double position, double velocity) {
    }

    public default void setOpenLoop(double voltage) {
    }

    public default void seed() {
    }

    public default void setSupplyCurrentLimit(double limitAmps) {
    }

    public default void setBrakeMode(boolean enable) {
    }

}

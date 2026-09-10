package frc.team4276.frc2026.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.SubsystemBase;

public class Feeder extends SubsystemBase {
    public enum SystemState {
        IDLE(-2.0),
        STOPPED(0.0),
        FEED(5.0);

        private final double voltage;

        SystemState(double voltage) {
            this.voltage = voltage;
        }

        public double getVoltage() {
            return voltage;
        }
    }

    private SystemState systemState = SystemState.IDLE;

    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
    private final FeederIO io;

    public Feeder(FeederIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        io.setOpenLoop(systemState.getVoltage());

        Logger.recordOutput("Feeder/SystemState", systemState);
    }

    public void setSystemState(SystemState state){
        systemState = state;
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }
}

package frc.team4276.frc2026.subsystems.spindexer;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.SubsystemBase;

public class Spindexer extends SubsystemBase {
    public enum SystemState {
        IDLE(-2.0),
        STOPPED(0.0),
        GOGOGO(10.0);

        private final double voltage;

        SystemState(double voltage){
            this.voltage = voltage;
        }

        public double getVoltage(){
            return voltage;
        }
    }

    private SystemState systemState = SystemState.IDLE;

    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
    private final SpindexerIO io;
    public Spindexer(SpindexerIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);

        io.setOpenLoop(systemState.getVoltage());

        Logger.recordOutput("Spindexer/SystemState", systemState);
    }

    public void setSystemState(SystemState state){
        systemState = state;
    }
    
    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }
}

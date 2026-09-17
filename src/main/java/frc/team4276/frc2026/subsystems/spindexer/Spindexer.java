package frc.team4276.frc2026.subsystems.spindexer;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class Spindexer implements Mechanism {
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

        getRegisteredScheduler().addPeriodic(this::updateInputs);
        setDefaultCommand(applyStateCommand());
    }

    private void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);

        Logger.recordOutput("Spindexer/SystemState", systemState);
    }

    /** Continuously applies the voltage for the currently requested {@link SystemState}. */
    public Command applyStateCommand() {
        return runRepeatedly(() -> io.setOpenLoop(systemState.getVoltage()))
            .withPriority(Command.LOWEST_PRIORITY)
            .named("Spindexer[APPLY STATE]");
    }

    public void setSystemState(SystemState state){
        systemState = state;
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }
}

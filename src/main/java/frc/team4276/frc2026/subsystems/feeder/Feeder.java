package frc.team4276.frc2026.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class Feeder implements Mechanism {
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

        // Reading sensors and logging does not control the mechanism, so it runs as a
        // scheduler callback rather than a command. Sideloads run before commands each
        // tick, so inputs are always fresh by the time the default command applies them.
        getRegisteredScheduler().addPeriodic(this::updateInputs);

        // Driving the motor does control the mechanism, so it has to be a command. This
        // is the lowest possible priority so any other command can take the feeder.
        setDefaultCommand(applyStateCommand());
    }

    private void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        Logger.recordOutput("Feeder/SystemState", systemState);
    }

    /** Continuously applies the voltage for the currently requested {@link SystemState}. */
    public Command applyStateCommand() {
        return runRepeatedly(() -> io.setOpenLoop(systemState.getVoltage()))
            .withPriority(Command.LOWEST_PRIORITY)
            .named("Feeder[APPLY STATE]");
    }

    public void setSystemState(SystemState state){
        systemState = state;
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }
}

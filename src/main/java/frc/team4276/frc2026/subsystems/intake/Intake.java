package frc.team4276.frc2026.subsystems.intake;

import static frc.team4276.frc2026.subsystems.intake.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class Intake implements Mechanism {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public enum WantedState {
        IDLE,
        RETRACT,
        INTAKE,
        EXHAUST
    }

    public enum SystemState {
        IDLING,
        RETRACTED,
        INTAKING,
        EXHAUSTING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public Intake(IntakeIO io){
        this.io = io;

        getRegisteredScheduler().addPeriodic(this::updateInputs);
        setDefaultCommand(applyStateCommand());
    }

    private void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    /** Continuously advances the state machine and applies the resulting outputs. */
    public Command applyStateCommand() {
        return runRepeatedly(() -> {
            systemState = handleStateTransition();
            applyState();

            Logger.recordOutput("Intake/SystemState", systemState);
            Logger.recordOutput("Intake/DesiredState", wantedState);
        }).withPriority(Command.LOWEST_PRIORITY).named("Intake[APPLY STATE]");
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case RETRACT -> SystemState.RETRACTED;
            case INTAKE -> SystemState.INTAKING;
            case EXHAUST -> SystemState.EXHAUSTING;
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                io.setOpenLoop(idleVolts);

                break;

            case RETRACTED:
                io.setOpenLoop(idleVolts);
                io.setPosition(retractPosition);

                break;

            case INTAKING:
                io.setOpenLoop(intakeVolts);
                io.setPosition(deployPosition);

                break;
            case EXHAUSTING:
                io.setOpenLoop(exhaustVolts);
                io.setPosition(deployPosition);

                break;
        }
    }

    public void setWantedState(WantedState state){
        wantedState = state;
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }
}

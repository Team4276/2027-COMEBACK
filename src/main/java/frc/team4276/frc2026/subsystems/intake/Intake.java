package frc.team4276.frc2026.subsystems.intake;

import static frc.team4276.frc2026.subsystems.intake.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.SubsystemBase;

public class Intake extends SubsystemBase {
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
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        systemState = handleStateTransition();
        applyState();

        Logger.recordOutput("Intake/SystemState", systemState);
        Logger.recordOutput("Intake/DesiredState", wantedState);
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

package frc.team4276.frc2026.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command3.Mechanism;

public class Hood implements Mechanism {
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodIO io;

    public Hood(HoodIO io) {
        this.io = io;

        getRegisteredScheduler().addPeriodic(this::updateInputs);
    }

    private void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    public void setPositionVelocity(double position, double velocity){ // TODO: add software limit; add backup homing sequence
        io.setPositionVelocity(position, velocity);
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }

    public boolean atSetpoint(){ // TODO: impl
        return false;
    }
}

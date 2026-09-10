package frc.team4276.frc2026.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIO io;

    public Turret(TurretIO io){
        this.io = io;
        
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        
    }

    public void setPositionVelocity(Rotation2d rotation, double velocity){
        io.setPositionVelocity(toPosition(rotation), velocity);
    }

    private double toPosition(Rotation2d rotation){
        return 0.0;
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }

    public boolean atSetpoint(){ // TODO: impl
        return false;
    }
}

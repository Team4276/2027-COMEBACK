package frc.team4276.frc2026.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.SubsystemBase;
import frc.team4276.lib.dashboard.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {
    private final LoggedTunableNumber hoodRatio = new LoggedTunableNumber("Flywheel/HoodRatio", 1.0);

    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final FlywheelIO io;
    public Flywheel(FlywheelIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        
    }

    public void setVelocities(double primaryRPM, double hoodRPM){
        io.setRpm(primaryRPM, hoodRPM);
    }

    public void setVelocity(double RPM){
        io.setRpm(RPM, RPM * hoodRatio.getAsDouble());
    }

    public void setBrakeMode(boolean enabled){
        io.setBrakeMode(enabled);
    }

    public boolean atSetpoint(){ // TODO: impl
        return false;
    }
}

package frc.team4276.frc2026.shooter;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import frc.team4276.frc2026.shooter.ShotCalculator.ShootingParameters;

public class ShooterConstants {
    public static final double tolerance = 10; // rpm

    public static final Transform3d robotToTurret = Transform3d.kZero;

    public static enum ParamPreset {
        // Shooting Presets
        SHOWER(Rotation2d.kZero,
                        0.0,
                        0.0),
        SHUB(Rotation2d.kZero,
                        0.0,
                        0.0),
        SHERRY(Rotation2d.kZero,
                        0.0,
                        0.0),
        SHTEAL(Rotation2d.kZero,
                        0.0,
                        0.0),
                        
        // Other
        STOW(Rotation2d.kZero,
                        0.0,
                        0.0),
        TURTLE(Rotation2d.kZero,
                        0.0,
                        0.0);

        private final ShootingParameters params;

        ParamPreset(Rotation2d turretAngle, double hoodAngle, double flywheelSpeed){
            this.params = new ShootingParameters(true, turretAngle, 0.0, hoodAngle, 0.0, flywheelSpeed);
        }

        public ShootingParameters getParams(){
            return params;
        }
    }
}

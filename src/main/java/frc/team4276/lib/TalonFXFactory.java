package frc.team4276.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

/**
 * Creates FXTalons objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {
    public static NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    public static InvertedValue INVERT_VALUE = InvertedValue.CounterClockwise_Positive;
    public static double NEUTRAL_DEADBAND = 0.04;

    // create a TalonFX with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(int id) {
        return createDefaultTalon(id, true);
    }

    public static TalonFX createDefaultTalon(int id, boolean trigger_config) {
        var talon = createTalon(id);
        talon.getConfigurator().apply(getDefaultConfig());
        return talon;
    }

    public static TalonFX createPermanentFollowerTalon(
            int followerId, int masterId, boolean opposeMasterDirection) {
        final TalonFX talon = createTalon(followerId);
        talon.setControl(new Follower(masterId, opposeMasterDirection ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        return talon;
    }

    public static TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NEUTRAL_MODE;
        config.MotorOutput.Inverted = INVERT_VALUE;
        config.MotorOutput.DutyCycleNeutralDeadband = NEUTRAL_DEADBAND;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRotorOffset = 0;
        config.Feedback.SensorToMechanismRatio = 1;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        config.Audio.BeepOnBoot = true;

        return config;
    }

    private static TalonFX createTalon(int id) {
        TalonFX talon = new TalonFX(id);
        talon.clearStickyFaults();

        return talon;
    }
}
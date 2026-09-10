package frc.team4276.frc2026.subsystems.drive;

import static frc.team4276.lib.PhoenixUtil.*;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import frc.team4276.frc2026.Ports;

public class GyroIPigeon2 implements GyroIO {
  private final Pigeon2 gyro = new Pigeon2(Ports.PIGEON);

  private final StatusSignal<Angle> yawStatusSignal = gyro.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocityStatusSignal = gyro.getAngularVelocityZWorld();


  public GyroIPigeon2() {
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.getConfigurator().setYaw(0.0);
    yawStatusSignal.setUpdateFrequency(DriveConstants.odometryFrequency);
    BaseStatusSignal.setUpdateFrequencyForAll(50, yawVelocityStatusSignal);
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro.getYaw());
    registerSignals(false, yawStatusSignal, yawVelocityStatusSignal);
    tryUntilOk(5, () -> gyro.setYaw(0.0, 0.25));

  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = StatusSignal.refreshAll(
        yawStatusSignal,
        yawVelocityStatusSignal)
        .isOK();

    inputs.yawPosition = Rotation2d.fromDegrees(yawStatusSignal.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocityStatusSignal.getValueAsDouble());

    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions = yawPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromDegrees(-value))
        .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}

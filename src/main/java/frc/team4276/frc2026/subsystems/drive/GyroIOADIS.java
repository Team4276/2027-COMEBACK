package frc.team4276.frc2026.subsystems.drive;

import java.util.Queue;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.ADIS16470_IMU;

public class GyroIOADIS implements GyroIO {
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOADIS() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(gyro::getAngle);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();

    inputs.yawPosition = Rotation2d.fromDegrees(gyro.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRate());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}

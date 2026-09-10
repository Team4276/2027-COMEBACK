package frc.team4276.lib;

import org.wpilib.math.geometry.Transform3d;
import frc.team4276.frc2026.subsystems.vision.VisionIO.VisionObservationType;

public class CameraConfig {
  public final String name;
  public final Transform3d robotToCamera;
  public final VisionObservationType observationType;

  public CameraConfig(String name, Transform3d robotToCamera, VisionObservationType observationType) {
    this.name = name;
    this.robotToCamera = robotToCamera;
    this.observationType = observationType;
  }
}

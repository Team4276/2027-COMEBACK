// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team4276.frc2026.subsystems.vision;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;
import frc.team4276.frc2026.Constants;
import frc.team4276.frc2026.Constants.Mode;
import frc.team4276.frc2026.subsystems.vision.VisionIO.VisionObservationType;
import frc.team4276.lib.CameraConfig;

import java.util.ArrayList;
import java.util.List;

public class VisionConstants {
  private static final boolean forceEnableInstanceLogging = false;
  public static final boolean enableInstanceLogging =
      forceEnableInstanceLogging || Constants.getMode() == Mode.REPLAY;

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Robot to camera transforms
  public static final Transform3d ov9281RobotToCamera = new Transform3d(
      Units.inchesToMeters(-1.0),
      Units.inchesToMeters(9.0),
      Units.inchesToMeters(8.0),
      new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(-20.0)));

  public static final Transform3d ov2311RobotToCamera = new Transform3d(
      Units.inchesToMeters(-1.0),
      Units.inchesToMeters(9.0) * -1.0,
      Units.inchesToMeters(8.0),
      new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(20.0)));

  public static final CameraConfig[] configs = new CameraConfig[] {
      new CameraConfig("Arducam_OV9281_USB_Camera", ov9281RobotToCamera, VisionObservationType.APRILTAG),
      new CameraConfig("Arducam_OV2311_USB_Camera", ov2311RobotToCamera, VisionObservationType.APRILTAG)
  };

  // Basic filtering thresholds
  public static final double maxAmbiguity = 0.19;
  public static final double maxZError = 0.3;
  public static final double maxSingleTagDistanceMeters = 3.0;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Vision can sometimes provide bad rotation updates
  public static final boolean useVisionRotation = true;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] cameraStdDevFactors = new double[] {
      1.0 // Camera 0
  };

  public static final double kLargeVariance = 1e6;

  public static final List<Integer> singleTagIdsToReject = new ArrayList<>() {
    {//TODO figure this out later lol; filter in subsystem instead of in the hardware impl
    }
  };
}
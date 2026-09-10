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

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.command2.SubsystemBase;
import frc.team4276.frc2026.FieldConstants;
import frc.team4276.frc2026.RobotState;
import frc.team4276.frc2026.RobotState.FuelTxTyObservation;
import frc.team4276.frc2026.subsystems.vision.VisionIO.PoseObservation;

import static frc.team4276.frc2026.subsystems.vision.VisionConstants.*;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

// TODO multicamera fusion
/*
 * multicamera fusion: idk what else to say; its just inverse variance weighting lol
 * put rot into vector form for fusion
 * 
 */

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] = new Alert(
          "Vision camera " + Integer.toString(i) + " is disconnected.", Level.MEDIUM);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        robotPoses.add(observation.pose());

        var estimate = processPoseObservation(observation);

        if (estimate.isEmpty()) {
          estimate = fuseGyro(observation);
        }

        // Add pose to log
        if (estimate.isEmpty()) {
          robotPosesRejected.add(observation.pose());

          continue;
        }

        robotPosesAccepted.add(observation.pose());

        // Calculate standard deviations
        // TODO: test poofs stdev factor method
        // double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double stdDevFactor = observation.tagCount() > 1 ? 1.0 : 1.0 / (1.0 - observation.ambiguity());
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        if (!useVisionRotation || (observation.tagCount() == 1)) {
          angularStdDev = kLargeVariance;
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      for (int i = 0; i < inputs[cameraIndex].objectFrames.length; i++) {
        double[] x = new double[4];
        double[] y = new double[4];

        for (int j = 0; j < 4; j++) {
          x[j] = inputs[cameraIndex].objectFrames[i][j];
          y[j] = inputs[cameraIndex].objectFrames[i][j + 4];
        }

        RobotState.getInstance().addObjectMeasurement(
            new FuelTxTyObservation(cameraIndex,
                x,
                y,
                inputs[cameraIndex].objectTimestamps[i]));
      }

      if (enableInstanceLogging) {
        // Log camera datadata
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[tagPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[robotPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      }
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  /** All criteria to reject a post returned by the vision system */
  private Optional<VisionFieldPoseEstimate> processPoseObservation(PoseObservation observation) {
    // Should have at least one tag
    if (observation.tagCount() <= 0) {
      return Optional.empty();
    }

    // Single tag results can provide more errors than multi-tag
    if (observation.tagCount() == 1) {
      if (observation.ambiguity() > maxAmbiguity) {
        return Optional.empty();
      }

      // Results get worse with a single tag at distance
      if (observation.averageTagDistance() > maxSingleTagDistanceMeters) {
        return Optional.empty();
      }
    }

    // Must have realistic Z coordinate, i.e. reject poses which think the robot is
    // hoving above
    // or below the floor.
    if (Math.abs(observation.pose().getZ()) > maxZError) {
      return Optional.empty();
    }

    if (observation.pose().getX() < 0.0
        || observation.pose().getX() > aprilTagLayout.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
      return Optional.empty();
    }

    return Optional.of(
        new VisionFieldPoseEstimate(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(0.0, 0.0, 0.0),
            observation.tagCount()));
  }

  private Optional<VisionFieldPoseEstimate> fuseGyro(PoseObservation observation) {
    if (observation.timestamp() < RobotState.getInstance().getLastUsedVisionPoseEstimateTimestamp()) {
      return Optional.empty();
    }

    var priorPose = RobotState.getInstance().getEstimatedOdomPoseAtTime(observation.timestamp());
    if (priorPose.isEmpty()) {
      return Optional.empty();
    }

    var maybeFieldToTag = FieldConstants.apriltagLayout.getTagPose(observation.firstTagId());
    if (maybeFieldToTag.isEmpty()) {
      return Optional.empty();
    }

    Pose2d fieldToTag = new Pose2d(maybeFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);

    Pose2d robotToTag = fieldToTag.relativeTo(observation.pose().toPose2d());

    Pose2d posteriorPose = new Pose2d(
        fieldToTag
            .getTranslation()
            .minus(
                robotToTag
                    .getTranslation()
                    .rotateBy(priorPose.get().getRotation())),
        priorPose.get().getRotation());

    return Optional.of(
        new VisionFieldPoseEstimate(
            posteriorPose,
            observation.timestamp(),
            VecBuilder.fill(0.0, 0.0, 0.0),
            observation.tagCount()));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
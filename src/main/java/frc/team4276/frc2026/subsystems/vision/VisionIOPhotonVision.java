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

import org.wpilib.math.geometry.Transform3d;

import static frc.team4276.frc2026.subsystems.vision.VisionConstants.*;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final VisionObservationType observationType;

  private final PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name             The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(int index) {
    camera = new PhotonCamera(configs[index].name);
    this.robotToCamera = configs[index].robotToCamera;
    this.observationType = configs[index].observationType;

    poseEstimator = new PhotonPoseEstimator(aprilTagLayout, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      if (observationType == VisionObservationType.FUEL) {
        inputs.objectFrames = new double[result.targets.size()][];
        
        for (int i = 0; i < result.targets.size(); i++) {
          inputs.objectFrames[i] = new double[8];
          for (int j = 0; j < 4; j++) {
            inputs.objectFrames[i][j] = result.targets.get(i).getDetectedCorners().get(j).x;
            inputs.objectFrames[i][j + 4] = result.targets.get(i).getDetectedCorners().get(j).y;
          }
        }

      } else if (observationType == VisionObservationType.APRILTAG) {
        Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);

        if (estimate.isEmpty()) {
          estimate = poseEstimator.estimateLowestAmbiguityPose(result);
        }

        estimate.ifPresent(
            (poseEstimate) -> {
              if (rejectEstimate(poseEstimate)) {
                return;
              }

              // Calculate average tag distance
              double totalTagDistance = 0.0;
              for (var target : poseEstimate.targetsUsed) {
                // Add tag IDs
                tagIds.add((short) target.fiducialId);
                totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
              }

              // Add observation
              poseObservations.add(
                  new PoseObservation(
                      poseEstimate.timestampSeconds, // Timestamp
                      poseEstimate.estimatedPose, // 3D pose estimate
                      poseEstimate.targetsUsed.get(0).poseAmbiguity, // Ambiguity
                      poseEstimate.targetsUsed.size(), // Tag count
                      poseEstimate.targetsUsed.get(0).fiducialId, // 1st tag id for single tag gyro fusion
                      totalTagDistance / poseEstimate.targetsUsed.size(), // Average tag distance
                      PoseObservationType.PHOTONVISION)); // Observation type
            });
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  private boolean rejectEstimate(EstimatedRobotPose estimate) {
    return false;
  }
}
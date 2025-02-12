/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public record PoseEstimate(EstimatedRobotPose pose, PhotonPipelineResult relevantResult) {}

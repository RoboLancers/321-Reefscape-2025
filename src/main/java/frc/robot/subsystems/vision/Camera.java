/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

@Logged
public class Camera {
  // for logging
  private final String name;

  private final CameraUsage usage;

  private final PhotonCamera camera;

  private final PhotonPoseEstimator multiTagEstimator;

  private final PhotonPoseEstimator singleTagEstimator;

  private Supplier<Rotation3d> heading;

  // for logging
  private VisionEstimate latestValidEstimate;

  public Camera(CameraConfig config, PhotonCamera camera, Supplier<Rotation3d> heading) {
    this.name = config.cameraName();

    this.usage = config.usage();

    this.camera = camera;

    this.multiTagEstimator =
        new PhotonPoseEstimator(
            RobotConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());

    this.singleTagEstimator =
        new PhotonPoseEstimator(
            RobotConstants.kAprilTagFieldLayout,
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            config.robotToCamera());

    this.heading = heading;
  }

  @NotLogged
  public VisionEstimate tryLatestEstimate() {
    singleTagEstimator.addHeadingData(Timer.getFPGATimestamp(), heading.get());

    if (!camera.isConnected()) return null;

    final var unreadResults = camera.getAllUnreadResults();

    if (unreadResults.isEmpty()) return null;

    final var latestResult = unreadResults.get(unreadResults.size() - 1);

    if (!latestResult.hasTargets()) return null;

    double poseAmbiguity = latestResult.getBestTarget().getPoseAmbiguity();

    final var multiTagEstimate =
        multiTagEstimator
            .update(latestResult)
            .filter(
                poseEst ->
                    VisionConstants.kAllowedFieldArea.contains(
                        poseEst.estimatedPose.getTranslation().toTranslation2d()))
            .map(
                photonEst -> {
                  final var visionEst =
                      new VisionEstimate(
                          photonEst,
                          calculateStdDevs(photonEst, EstimateType.MULTI_TAG, poseAmbiguity),
                          name,
                          usage,
                          EstimateType.MULTI_TAG);

                  return visionEst;
                })
            .orElse(null);

    final var singleTagEstimate =
        singleTagEstimator
            .update(latestResult)
            .filter(
                poseEst ->
                    VisionConstants.kAllowedFieldArea.contains(
                        poseEst.estimatedPose.getTranslation().toTranslation2d()))
            .map(
                photonEst -> {
                  final var visionEst =
                      new VisionEstimate(
                          photonEst,
                          calculateStdDevs(photonEst, EstimateType.SINGLE_TAG, poseAmbiguity),
                          name,
                          usage,
                          EstimateType.SINGLE_TAG);

                  return visionEst;
                })
            .orElse(null);

    return switch (latestResult.targets.size()) {
      case 1 -> singleTagEstimate;
      default -> multiTagEstimate;
    };
  }

  // could be absolute nonsense, open to tuning constants for each robot camera config
  // assumes `result` has targets
  @NotLogged
  private Matrix<N3, N1> calculateStdDevs(
      EstimatedRobotPose visionPoseEstimate, EstimateType estimateType, double poseAmbiguity) {
    // weighted average by distance
    if (poseAmbiguity > 0.4 && estimateType == EstimateType.SINGLE_TAG)
      return VecBuilder.fill(
          Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    double rotationStdDevMultiplier =
        estimateType != EstimateType.SINGLE_TAG ? 1 : VisionConstants.kSingleTagStdDevCoeff;

    final double avgTargetDistance =
        visionPoseEstimate.targetsUsed.stream()
                .mapToDouble(
                    t -> {
                      final double bestCameraToTargetDistance =
                          t.getBestCameraToTarget()
                              .getTranslation()
                              .getDistance(new Translation3d());

                      return bestCameraToTargetDistance;
                    })
                .reduce(0.0, Double::sum)
            / (visionPoseEstimate.targetsUsed.size());

    final double translationStdDev =
        VisionConstants.kTranslationStdDevCoeff
            * Math.pow(avgTargetDistance, 3)
            / Math.pow(visionPoseEstimate.targetsUsed.size(), 3);
    final double rotationStdDev =
        VisionConstants.kRotationStdDevCoeff
            * rotationStdDevMultiplier
            * Math.pow(avgTargetDistance, 3)
            / Math.pow(visionPoseEstimate.targetsUsed.size(), 3);

    return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
  }

  public boolean isConnected() {
    return camera.isConnected();
  }
}

/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private final Supplier<Rotation2d> robotHeadingSupplier;

  private final PhotonPoseEstimator multiTagEstimator;

  private final PhotonPoseEstimator singleTagEstimator;

  private final CameraConfig config;

  // for logging
  private VisionEstimate latestMultiTagEstimate;

  private VisionEstimate latestSingleTagEstimate;

  public Camera(
      CameraConfig config, PhotonCamera camera, Supplier<Rotation2d> robotHeadingSupplier) {

    this.config = config;

    this.name = config.cameraName();

    this.usage = config.usage();

    this.camera = camera;

    this.robotHeadingSupplier = robotHeadingSupplier;

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
  }

  public boolean isReefCamera() {
    return usage == CameraUsage.REEF;
  }

  public boolean canSeeTag(int id) {
    final var unreadResults = camera.getAllUnreadResults();

    if (unreadResults.isEmpty()) return false;

    final var latestResult = unreadResults.get(unreadResults.size() - 1);

    if (!latestResult.hasTargets()) return false;

    return latestResult.targets.stream().anyMatch(target -> target.fiducialId == id);
  }

  public VisionEstimate getLatestMultiTagEstimate() {
    return latestMultiTagEstimate;
  }

  public VisionEstimate getLatestSingleTagEstimate() {
    return latestSingleTagEstimate;
  }

  @NotLogged
  public VisionEstimate tryLatestEstimate() {
    singleTagEstimator.addHeadingData(Timer.getFPGATimestamp(), robotHeadingSupplier.get());

    if (!camera.isConnected()) return null;

    final var unreadResults = camera.getAllUnreadResults();

    if (unreadResults.isEmpty()) return null;

    final var latestResult = unreadResults.get(unreadResults.size() - 1);

    if (!latestResult.hasTargets()) return null;

    final var multiTagEstimate =
        multiTagEstimator
            .update(latestResult)
            .filter(
                poseEst ->
                    VisionConstants.kAllowedFieldArea.contains(
                            poseEst.estimatedPose.getTranslation().toTranslation2d())
                        && poseEst
                            .estimatedPose
                            .getMeasureZ()
                            .isNear(Meters.zero(), VisionConstants.kAllowedFieldHeight))
            .map(
                photonEst -> {
                  final var visionEst =
                      new VisionEstimate(
                          photonEst,
                          calculateStdDevs(photonEst, EstimateType.MULTI_TAG),
                          name,
                          usage,
                          EstimateType.MULTI_TAG);

                  latestMultiTagEstimate = visionEst;

                  return visionEst;
                })
            .orElse(null);

    final var singleTagEstimate =
        singleTagEstimator
            .update(latestResult)
            .filter(
                poseEst ->
                    VisionConstants.kAllowedFieldArea.contains(
                            poseEst.estimatedPose.getTranslation().toTranslation2d())
                        && poseEst
                            .estimatedPose
                            .getMeasureZ()
                            .isNear(Meters.zero(), VisionConstants.kAllowedFieldHeight))
            .map(
                photonEst -> {
                  final var visionEst =
                      new VisionEstimate(
                          photonEst,
                          calculateStdDevs(photonEst, EstimateType.SINGLE_TAG),
                          name,
                          usage,
                          EstimateType.SINGLE_TAG);

                  latestSingleTagEstimate = visionEst;

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
      EstimatedRobotPose visionPoseEstimate, EstimateType estimateType) {
    // weighted average by ambiguity
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

    final double estimateTypeMultiplier =
        (estimateType == EstimateType.SINGLE_TAG) ? VisionConstants.kSingleTagStdDevMultiplier : 1;

    final double targetDistancePower =
        (estimateType == EstimateType.SINGLE_TAG)
            ? VisionConstants.kSingleTagTargetDistancePower
            : VisionConstants.kMultiTagTargetDistancePower;

    if (visionPoseEstimate.targetsUsed.get(0).poseAmbiguity > VisionConstants.kAmbiguityThreshold
        && estimateType == EstimateType.SINGLE_TAG) {
      return null;
    }

    double poseAmbiguityMultiplier =
        estimateType != EstimateType.SINGLE_TAG
            ? 1
            : Math.max(
                1,
                (visionPoseEstimate.targetsUsed.get(0).poseAmbiguity
                        + VisionConstants.kAmbiguityShifter)
                    * VisionConstants.kAmbiguityScalar);

    final double translationStdDev =
        estimateTypeMultiplier
            * VisionConstants.kTranslationStdDevCoeff
            * Math.pow(avgTargetDistance, targetDistancePower)
            * poseAmbiguityMultiplier
            / Math.pow(visionPoseEstimate.targetsUsed.size(), 3);

    final double rotationStdDev =
        estimateTypeMultiplier
            * VisionConstants.kRotationStdDevCoeff
            * Math.pow(avgTargetDistance, targetDistancePower)
            * poseAmbiguityMultiplier
            / Math.pow(visionPoseEstimate.targetsUsed.size(), 3);

    return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
  }

  public boolean isConnected() {
    return camera.isConnected();
  }
}

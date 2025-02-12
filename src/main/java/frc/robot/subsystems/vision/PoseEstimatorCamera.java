/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionTargetSim;

// abstraction for multicamera setup
@Logged
public class PoseEstimatorCamera {
  private final String name; // solely for logging

  private final CameraIO io;
  private final CameraInputs inputs;

  private final PhotonPoseEstimator poseEstimator;

  private PoseEstimate latestEstimate;

  public static PoseEstimatorCamera createSim(
      CameraConfig config,
      PhotonCameraSim sim,
      Supplier<Pose3d> cameraPoseSupplier,
      Set<VisionTargetSim> targets) {
    return new PoseEstimatorCamera(new CameraIOSim(sim, cameraPoseSupplier, targets), config);
  }

  public static PoseEstimatorCamera createReal(CameraConfig config) {
    return new PoseEstimatorCamera(new CameraIOReal(config), config);
  }

  private PoseEstimatorCamera(CameraIO io, CameraConfig config) {
    this.name = config.cameraName();

    this.io = io;
    this.inputs = new CameraInputs();

    this.poseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());
  }

  public void update() {
    io.updateInputs(inputs);

    final Optional<EstimatedRobotPose> optionalEstimate = poseEstimator.update(inputs.latestResult);

    optionalEstimate.ifPresent(est -> latestEstimate = new PoseEstimate(est, inputs.latestResult));
  }

  // for use by drivetrain pose estimator, may consolidate into a robot-wide pose estimator
  public PoseEstimate getLatestEstimate() {
    return latestEstimate;
  }
}

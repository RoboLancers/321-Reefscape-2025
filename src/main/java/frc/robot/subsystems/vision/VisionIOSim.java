/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
  private final VisionSystemSim vision;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final List<PoseEstimatorCamera> cameras;

  public VisionIOSim(Supplier<Pose2d> robotPoseSupplier, CameraConfig... configs) {
    this.vision = new VisionSystemSim("main");

    vision.addAprilTags(VisionConstants.kAprilTagFieldLayout);

    this.robotPoseSupplier = robotPoseSupplier;

    this.cameras = new ArrayList<>();

    for (final CameraConfig config : configs) {
      SimCameraProperties properties = null; // fail early
      try {
        properties =
            new SimCameraProperties(
                config.jsonConfigPath(), config.resolutionWidth(), config.resolutionHeight());
      } catch (IOException e) {
        e.printStackTrace();
      }

      final var simCamera = new PhotonCameraSim(new PhotonCamera(config.cameraName()), properties);

      vision.addCamera(simCamera, config.robotToCamera());

      cameras.add(
          PoseEstimatorCamera.createSim(
              config,
              simCamera,
              () -> new Pose3d(robotPoseSupplier.get()).plus(config.robotToCamera()),
              vision.getVisionTargets()));
    }
  }

  @Override
  public void updateEstimates(BiConsumer<EstimatedRobotPose, Matrix<N3, N1>> visionDataConsumer) {
    vision.update(robotPoseSupplier.get());

    cameras.forEach(PoseEstimatorCamera::update);

    for (final var visionEst :
        cameras.stream().map(PoseEstimatorCamera::getLatestEstimate).toList()) {
      visionDataConsumer.accept(
          visionEst.pose(), Vision.calculateStdDevs(visionEst.relevantResult()));
    }
  }
}
